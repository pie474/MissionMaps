#include <iostream>
#include <fstream>
#include <sstream>
#include <SFML/Graphics.hpp>
#include <list>
#include <cmath>
#include <functional>
#include <queue>
#include <unordered_set>
#include <stack>
using namespace std;

/**
 * stores a wall that spans between 2 points
 */
struct Wall
{
    sf::Vector2f a, b;
};

/**
 * an individual node on the graph - stores a position, as well as various metrics used in A*
 */
struct MapNode
{
    sf::Vector2f pos;
    list<MapNode*> neighbors;
    double cost = -1, heuristic_cost = -1;
    bool visited = false;
    MapNode* previous = nullptr;
};

MapNode *start_node = nullptr, *end_node = nullptr;

/**
 * calculates distance between 2 points
 * @param A
 * @param B
 * @return distance
 */
double distance(const MapNode& A, const MapNode& B)
{
    sf::Vector2f diff = B.pos - A.pos;
    return sqrt(diff.x*diff.x + diff.y*diff.y);
}


list<Wall> obstacles; //stores individual walls, whether part of a building or not
list<MapNode> school_graph; //stores all pathable nodes


//NOTE: make sure to add all obstacles BEFORE creating graph; this does NOT disrupt existing node links
/**
 * adds a wall between two points to the map
 * @param A
 * @param B
 */
void addObstacle(sf::Vector2f A, sf::Vector2f B)
{
    Wall wall;
    wall.a = A;
    wall.b = B;
    obstacles.push_back(wall);
}


//FROM https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool onSegment(sf::Vector2f p, sf::Vector2f q, sf::Vector2f r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
    q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;

    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(sf::Vector2f p, sf::Vector2f q, sf::Vector2f r)
{
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    float val = (q.y - p.y) * (r.x - q.x) -
            (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // collinear TODO: use threshold instead of exact check? may not be necessary

    return (val > 0)? 1: 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(sf::Vector2f p1, sf::Vector2f q1, sf::Vector2f p2, sf::Vector2f q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

/**
 * checks whether a particular wall is in between two points
 * @param A
 * @param B
 * @param wall
 * @return whether the wall obstructs the path between the two points
 */
bool isObstructed(sf::Vector2f A, sf::Vector2f B, Wall wall)
{
    return doIntersect(A, B, wall.a, wall.b);
}

/**
 * checks whether two points are obstructed by an obstacle in the map
 * @param A
 * @param B
 * @return whether the line between them is obstructed
 */
bool isObstructed(sf::Vector2f A, sf::Vector2f B)
{
    list<Wall>::iterator it;
    for(it = obstacles.begin(); it != obstacles.end(); ++it)
    {
        if(isObstructed(A, B, *it))
        {
            return true;
        }
    }
    return false;
}

/**
 * inputs new node into the pathable graph
 * @param point the position to add to the graph
 * @return pointer to new node
 */
MapNode* addNode(sf::Vector2f point)
{
    MapNode new_node = MapNode();
    new_node.pos = point;

    school_graph.push_back(new_node);

    MapNode* new_node_pointer = &school_graph.back();

    list<MapNode>::iterator iterator;
    for(iterator = school_graph.begin(); iterator != school_graph.end(); ++iterator)
    {
        if(&*iterator != new_node_pointer && !isObstructed(point, iterator->pos))//if it's possible to physically go between 2 nodes, create link between them
        {
            new_node_pointer->neighbors.push_back(&(*iterator));
            iterator->neighbors.push_back(new_node_pointer);
        }
    }
    return new_node_pointer;
}


/**
 * draws out the map graph and obstacles to the screen
 * @param win the window to draw it to
 */
void display_map(sf::RenderWindow& win)
{
    list<Wall>::iterator obstacle_iterator;
    for(obstacle_iterator = obstacles.begin(); obstacle_iterator != obstacles.end(); ++obstacle_iterator)
    {
        //draw line to neighbor
        sf::Vertex line[2];
        line[0] = sf::Vertex(obstacle_iterator->a, sf::Color::Red);
        line[1] = sf::Vertex(obstacle_iterator->b, sf::Color::Red);
        win.draw(line, 2, sf::Lines);
    }

    list<MapNode>::iterator map_iterator;
    for(map_iterator = school_graph.begin(); map_iterator != school_graph.end(); ++map_iterator)
    {
        sf::Vertex node_vertex = sf::Vertex(map_iterator->pos, sf::Color::Yellow);
        list<MapNode*>::iterator neighbor_iterator;
        for(neighbor_iterator = map_iterator->neighbors.begin(); neighbor_iterator != map_iterator->neighbors.end(); ++neighbor_iterator)
        {
            //line to neighbor
            sf::Vertex line[2];
            line[0] = sf::Vertex((*neighbor_iterator)->pos, sf::Color::Yellow);
            line[1] = node_vertex;
            win.draw(line, 2, sf::Lines);
        }

        //draw node
        sf::CircleShape node_circle = sf::CircleShape();
        node_circle.setRadius(5);
        node_circle.setPosition(map_iterator->pos);
        node_circle.setFillColor(sf::Color::White);
        win.draw(node_circle);
    }
}



//  A* stuff
//     |
//     |
//     V


/**
 * the heuristic cost function for A*
 * @param node the node to calculate the cost for
 * @return the cost
 */
double heuristic_cost(const MapNode* node)
{
    return distance(*node, *end_node);
}

struct compare_cost
{
    bool operator() ( const MapNode* a, const  MapNode* b) const
        { return a->heuristic_cost > b->heuristic_cost ; }
};

priority_queue<MapNode*, vector<MapNode*>, compare_cost> frontier_queue;
unordered_set<MapNode*> frontier_set = unordered_set<MapNode*>();

/**
 * resets all data from potential previous A* runs
 */
void reset()
{
    list<MapNode>::iterator map_iterator;
    for(map_iterator = school_graph.begin(); map_iterator != school_graph.end(); ++map_iterator)
    {
        map_iterator->visited = false;
        map_iterator->cost = -1;
        map_iterator->heuristic_cost = 0;
        map_iterator->previous = nullptr;
    }

    //empty frontier
    while(!frontier_queue.empty())
    {
        frontier_queue.pop();
    }
    frontier_set.clear();
}


/**
 * checks whether the frontier contains a node
 * @param node the node to look for
 * @return whether or not the node exists in the frontier
 */
bool frontier_contains(MapNode* node)
{
    return frontier_set.find(node) != frontier_set.end();
}

/**
 * adds a map node to the frontier
 * @param node the node to add
 */
void add_to_frontier(MapNode* node)
{
    frontier_queue.push(node);
    frontier_set.insert(node);
}

/**
 * prints out everything in the current frontier for debugging purposes
 */
void print_frontier()
{
    stack<MapNode*> temp_storage = stack<MapNode*>();
    while(!frontier_queue.empty())
    {
        cout << heuristic_cost(frontier_queue.top()) << " ";
        temp_storage.push(frontier_queue.top());
        frontier_queue.pop();
    }
    cout << endl;
    while(!temp_storage.empty())
    {
        frontier_queue.push(temp_storage.top());
        temp_storage.pop();
    }
}

/**
 * Uses A* search algorithm to find the shortest path between start and end node
 * updates path pointers on every node in graph
 */
void find_path()
{
    reset();
    add_to_frontier(start_node);
    //start_node->cost = 0;

    while(!frontier_contains(end_node))
    {
        //print_frontier();

        MapNode* current_node = frontier_queue.top();
        frontier_set.erase(current_node);
        frontier_queue.pop();

        current_node->visited = true;

        list<MapNode*>::iterator neighbor_iterator;
        for(neighbor_iterator = current_node->neighbors.begin(); neighbor_iterator != current_node->neighbors.end(); ++neighbor_iterator)
        {
            MapNode* neighbor = *neighbor_iterator;

            if(neighbor->visited)
                continue;

            if(!frontier_contains(neighbor))
            {
                neighbor->previous = current_node;
                neighbor->heuristic_cost = heuristic_cost(neighbor);
                neighbor->cost = current_node->cost + distance(current_node, neighbor);
                add_to_frontier(neighbor);
            }
            else if(current_node->cost + distance(current_node, neighbor) < neighbor->cost)
            {
                neighbor->previous = current_node;
                neighbor->heuristic_cost = heuristic_cost(neighbor);
                neighbor->cost = current_node->cost + distance(current_node, neighbor);
            }
        }
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "Mission Maps");
    window.setActive();
    window.setFramerateLimit(30);

    //load map
    string line;
    ifstream map_file("../map.txt");
    if (map_file.is_open())
    {
        while ( getline (map_file, line) )//for each line in file
        {
            cout << line << '\n';
            if(line.length() <3) continue;
            char cmd = line[0];
            line.erase(0, 2);

            istringstream line_stream;
            line_stream.str(line);
            float x, y, x2, y2;
            switch(cmd) {
                case '#': //comment, so do nothing
                    break;
                case 'o': //obstacle
                    line_stream >> x >> y >> x2 >> y2;
                    addObstacle(sf::Vector2f(x, y), sf::Vector2f(x2, y2));
                    break;
                case 'n': //normal node
                    line_stream >> x >> y;
                    addNode(sf::Vector2f(x, y));
                    break;
                case 's': //start node
                    line_stream >> x >> y;
                    start_node = addNode(sf::Vector2f(x, y));
                    break;
                case 'e': //end node
                    line_stream >> x >> y;
                    end_node = addNode(sf::Vector2f(x, y));
                    break;
                default:
                    cout << "UNKNOWN CMD: " << cmd << " (" << line << ")\n";
            }
        }
        map_file.close();
    }
    else
    {
        cout << "Unable to open map file";
        return 1;
    }


//    addObstacle(sf::Vector2f(200, 500), sf::Vector2f(600, 500));
//    addObstacle(sf::Vector2f(0, 700), sf::Vector2f(300, 700));
//    addObstacle(sf::Vector2f(500, 700), sf::Vector2f(1000, 700));

//    addNode(sf::Vector2f(50, 50));
//    addNode(sf::Vector2f(850, 850));
//    addNode(sf::Vector2f(850, 450));
//    addNode(sf::Vector2f(450, 400));

    sf::Clock clock;

    while(window.isOpen())
    {
        sf::Event event{};

        while(window.pollEvent(event))
        {
            if(event.type == sf::Event::Closed)
                window.close();
            else if(event.type == sf::Event::MouseButtonPressed)
            {
                addNode(sf::Vector2f(sf::Mouse::getPosition(window)));
            }
            else if(event.type == sf::Event::KeyPressed)
            {
                if(event.key.code == sf::Keyboard::Escape)
                {
                    window.close();
                    return 0;
                }
                clock.restart();
                find_path();
                cout << "path found in " << clock.getElapsedTime().asSeconds() << " seconds\n";
            }
        }


        window.clear(sf::Color::Black);

        display_map(window);

        //display path if path exists
        if(end_node != nullptr && end_node->previous != nullptr)
        {
            MapNode* curr = end_node;
            while(curr != start_node)
            {
                sf::Vertex line[2];
                line[0] = sf::Vertex(curr->pos, sf::Color::Blue);
                line[1] = sf::Vertex(curr->previous->pos, sf::Color::Blue);
                window.draw(line, 2, sf::Lines);
                curr = curr->previous;
            }
        }


        window.display();


    }
    return 0;
}
