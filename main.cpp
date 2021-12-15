#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <SFML/Graphics.hpp>

using namespace std;

// Structs for Walls and Nodes
/**
 * Stores a wall that spans between 2 points
 */
struct Wall
{
    sf::Vector2f a;
    sf::Vector2f b;
};

/**
 * Individual node on the graph
 * Stores position and Dijkstra's algorithm metrics
 */
struct MapNode
{
    sf::Vector2f pos;
    list<MapNode*> neighbors;
    double cost = 1000000, heuristic_cost = -1;
    bool visited = false;
    MapNode* previous = nullptr;
    string name = "";
};

// Variables for Walls and Nodes
MapNode *start_node = nullptr, *end_node = nullptr;
list<Wall> walls; // stores all walls
list<MapNode> school_graph; // stores all nodes

// Functions for Obstruction Checking

/**
 * Finds orientation of ordered triplet (p, q, r)
 * Helper function for doIntersect()
 * Source: https://www.geeksforgeeks.org/orientation-3-ordered-points/
 * @param p
 * @param q
 * @param r
 * @return orientation: 0 -> collinear, 1 -> CW, 2 -> CCW
 */
int orientation(sf::Vector2f p, sf::Vector2f q, sf::Vector2f r)
{
    float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0; // Collinear

    if (val > 0) return 1; // CW

    return 2; // CCW
}

/**
 * Checks if q lies on line segment pr
 * Helper function for doIntersect()
 * Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 * @param p
 * @param q
 * @param r
 * @return boolean whether q lies on line segment pr
 */
bool onSegment(sf::Vector2f p, sf::Vector2f q, sf::Vector2f r)
{
    return q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y);
}

/**
 * Determines if line segments p1q1 and p2q2 intersect
 * Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 * @param p
 * @param q
 * @param r
 * @return boolean whether line segments p1q1 and p2q2 intersect
 */
bool doIntersect(sf::Vector2f p1, sf::Vector2f q1, sf::Vector2f p2, sf::Vector2f q2)
{
    // Find the four orientations needed for general and special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General Case
    if (o1 != o2 && o3 != o4) return true;

    // Special Cases
    // p1 / q1 / p2 are collinear, p2 lies on p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1 / q1 / q2 are collinear, q2 lies on p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2 / q2 / p1 are collinear, p1 lies on p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2 / q2 / q1 are collinear, q1 lies on p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    // Doesn't fall into other cases
    return false;
}

/**
 * Checks whether a wall is in between two points
 * Helper function for isObstructed() without wall parameter
 * @param a
 * @param b
 * @param wall
 * @return boolean whether the wall obstructs the line segment connecting the points
 */
bool isObstructed(sf::Vector2f a, sf::Vector2f b, Wall wall)
{
    return doIntersect(a, b, wall.a, wall.b);
}

/**
 * Checks whether two points are obstructed by an obstacle in the map
 * @param a
 * @param b
 * @return boolean whether the line segment connecting the points is obstructed by any wall
 */
bool isObstructed(sf::Vector2f a, sf::Vector2f b)
{
    list<Wall>::iterator iterator;
    for (iterator = walls.begin(); iterator != walls.end(); ++iterator)
    {
        if (isObstructed(a, b, *iterator))
        {
            return true;
        }
    }
    return false;
}

// Functions for Adding Walls and Nodes

/**
 * Inputs new node with specified name into the pathable graph
 * Helper function for addNode() without name parameter
 * @param point the position to add to the graph
 * @return pointer to new node
 */
MapNode* addNode(sf::Vector2f point, string name)
{
    // Create Node
    MapNode new_node = MapNode();
    new_node.pos = point;
    new_node.name = name;

    // Add Node to Graph
    school_graph.push_back(new_node);

    // Get Node Pointer
    MapNode* new_node_pointer = &school_graph.back();

    // Link All Nodes to New Node if No Obstructions
    list<MapNode>::iterator iterator;
    for (iterator = school_graph.begin(); iterator != school_graph.end(); ++iterator)
    {
        if (&*iterator != new_node_pointer && !isObstructed(point, iterator->pos))
        {
            new_node_pointer->neighbors.push_back(&(*iterator));
            iterator->neighbors.push_back(new_node_pointer);
        }
    }

    return new_node_pointer;
}

/**
 * Inputs new node into the pathable graph
 * @param point the position to add to the graph
 * @return pointer to new node
 */
MapNode* addNode(sf::Vector2f point)
{
    return addNode(point, " ");
}

/**
 * adds a wall between two points to the map
 * @param a
 * @param b
 */
void addWall(sf::Vector2f a, sf::Vector2f b)
{
    Wall wall;
    wall.a = a;
    wall.b = b;
    walls.push_back(wall);
}

// Dijkstra Structs / Variables / Functions

/**
 * Custom comparator for MapNodes
 * Based on heuristic cost for both nodes
 */
struct compare_cost
{
    bool operator() (const MapNode* a, const  MapNode* b) const
    {
        return a->heuristic_cost > b->heuristic_cost;
    }
};

priority_queue<MapNode*, vector<MapNode*>, compare_cost> frontier_queue;
unordered_set<MapNode*> frontier_set = unordered_set<MapNode*>();

/**
 * Calculates distance between two points
 * @param a
 * @param b
 * @return distance
 */
double distance(const MapNode& a, const MapNode& b)
{
    sf::Vector2f diff = b.pos - a.pos;
    return hypot(diff.x, diff.y);
}

/**
 * Heuristic cost function for Dijkstra
 * @param node the node to calculate the cost for
 * @return cost
 */
double heuristic_cost(const MapNode* node)
{
    return distance(*node, *end_node);
}

/**
 * Resets all data from previous Dijkstra's runs
 */
void reset()
{
    // Reset All Nodes
    list<MapNode>::iterator map_iterator;
    for (map_iterator = school_graph.begin(); map_iterator != school_graph.end(); ++map_iterator)
    {
        map_iterator->visited = false;
        map_iterator->cost = 1000000;
        map_iterator->heuristic_cost = -1;
        map_iterator->previous = nullptr;
    }

    // Empty Frontier
    while (!frontier_queue.empty())
    {
        frontier_queue.pop();
    }
    frontier_set.clear();
}

/**
 * Uses Dijkstra's Algorithm to attempt to find the shortest path between start and end node
 * Updates path pointers on every node in graph
 */
void find_path_dijkstra()
{
    reset();
    list<MapNode>::iterator map_iterator;
    int num_nodes = 0;
    for (map_iterator = school_graph.begin(); map_iterator != school_graph.end(); ++map_iterator)
    {
        map_iterator->cost = 1000000;
        map_iterator->visited = false;
        num_nodes++;
    }
    start_node->cost = 0;

    // Loop Until Path is Completed
    while (num_nodes > 0)
    {
        // Visit Node with Least Cost
        MapNode* current_node = nullptr;
        for (map_iterator = school_graph.begin(); map_iterator != school_graph.end(); ++map_iterator)
        {
            if(!map_iterator->visited && (current_node == nullptr || (map_iterator->cost < current_node->cost)))
            {
                current_node = &*map_iterator;
            }
        }
        current_node->visited = true;
        num_nodes--;

        // Loop for Neighboring Nodes
        list<MapNode*>::iterator neighbor_iterator;
        for (neighbor_iterator = current_node->neighbors.begin(); neighbor_iterator != current_node->neighbors.end(); ++neighbor_iterator)
        {
            MapNode* neighbor = *neighbor_iterator;

            // Ignore if Neighbor is Already visited
            if (neighbor->visited)
            {
                continue;
            }

            double distance_through = current_node->cost + distance(*current_node, *neighbor);

            if (distance_through < neighbor->cost)
            {
                neighbor->cost = distance_through;
                neighbor->previous = current_node;
            }
        }
    }
}

// SFML Drawing Functions

bool DEBUG_UI = false;

/**
 * Draws a line with specified endpoints and thickness
 * @param win the window to draw it to
 * @param pt1 first endpoint
 * @param pt2 second endpoint
 * @param thickness line thickness
 * @param color line color
 */
void draw_line(sf::RenderWindow& win, sf::Vector2f pt1, sf::Vector2f pt2, double thickness, sf::Color color)
{
    sf::ConvexShape line = sf::ConvexShape();
    line.setPointCount(4);

    // Define Variables for Convenience
    double x1 = pt1.x;
    double y1 = pt1.y;
    double x2 = pt2.x;
    double y2 = pt2.y;

    double theta = atan2(y2 - y1, x2 - x1);
    double s = sin(theta);
    double c = cos(theta);
    double t = thickness / 2;

    // Defining Line Corner Points
    line.setPoint(0, sf::Vector2f(x1 + t*s, y1 - t*c));
    line.setPoint(1, sf::Vector2f(x2 + t*s, y2 - t*c));
    line.setPoint(2, sf::Vector2f(x2 - t*s, y2 + t*c));
    line.setPoint(3, sf::Vector2f(x1 - t*s, y1 + t*c));

    // Drawing Line
    line.setFillColor(color);
    win.draw(line);

    // Endpoint Circle 1
    sf::CircleShape circle1 = sf::CircleShape();
    circle1.setRadius(t);
    circle1.setPosition(sf::Vector2f(x1 - t, y1 - t));
    circle1.setFillColor(color);
    win.draw(circle1);

    // Endpoint Circle 2
    sf::CircleShape circle2 = sf::CircleShape();
    circle2.setRadius(t);
    circle2.setPosition(sf::Vector2f(x2 - t, y2 - t));
    circle2.setFillColor(color);
    win.draw(circle2);
}

/**
 * Draws nodes and walls
 * @param win the window to draw it to
 */
void display_map(sf::RenderWindow& win)
{
    // Draw Obstacles if Debug Mode
    if (DEBUG_UI)
    {
        list<Wall>::iterator obstacle_iterator;
        for (obstacle_iterator = walls.begin(); obstacle_iterator != walls.end(); ++obstacle_iterator)
        {
            // Draw Obstacle Line
            sf::Vertex line[2];
            line[0] = sf::Vertex(obstacle_iterator->a, sf::Color::Red);
            line[1] = sf::Vertex(obstacle_iterator->b, sf::Color::Red);
            win.draw(line, 2, sf::Lines);
        }
    }

    // Draw Intermediate Nodes if Debug Mode and Endpoint Modes
    list<MapNode>::iterator map_iterator;
    for (map_iterator = school_graph.begin(); map_iterator != school_graph.end(); ++map_iterator)
    {
        if (map_iterator->name != " " || DEBUG_UI)
        {
            sf::CircleShape node_circle = sf::CircleShape();
            node_circle.setRadius(5);
            node_circle.setPosition(map_iterator->pos - sf::Vector2f(5, 5));
            node_circle.setFillColor(sf::Color::White);
            win.draw(node_circle);
        }
    }
}

/**
 * Draws connections between nodes
 * @param win the window to draw it to
 */
void display_graph(sf::RenderWindow& win)
{
    // Loop for Every Node
    list<MapNode>::iterator map_iterator;
    for (map_iterator = school_graph.begin(); map_iterator != school_graph.end(); ++map_iterator)
    {
        sf::Vertex node_vertex = sf::Vertex(map_iterator->pos, sf::Color::Yellow);

        // Loop for Every Neighboring Node
        list<MapNode*>::iterator neighbor_iterator;
        for (neighbor_iterator = map_iterator->neighbors.begin(); neighbor_iterator != map_iterator->neighbors.end(); ++neighbor_iterator)
        {
            // Draw Line from Node to Neighboring Node
            sf::Vertex line[2];
            line[0] = sf::Vertex((*neighbor_iterator)->pos, sf::Color::Yellow);
            line[1] = node_vertex;
            win.draw(line, 2, sf::Lines);
        }
    }
}

int main() {
    // Load Files
    sf::Image icon = sf::Image();
    if (!icon.loadFromFile("../icon.png"))
    {
        cout << "Unable to open icon image" << endl;
        return 1;
    }

    sf::Texture background_image = sf::Texture();
    if (!background_image.loadFromFile("../blank_map.png"))
    {
        cout << "Unable to open background image" << endl;
        return 1;
    }
    sf::Sprite background = sf::Sprite(background_image);

    sf::Font ARIAL;
    if (!ARIAL.loadFromFile("../arial.ttf"))
    {
        cout << "Unable to load font" << endl;
        return 1;
    }

    ifstream map_file = ifstream("../mission.txt");
    if (!map_file.is_open())
    {
        cout << "Unable to open map file" << endl;
        return 1;
    }

    // Create Window
    sf::RenderWindow window(sf::VideoMode(1418, 1221), "Mission Maps");
    window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());
    window.setActive();
    window.setFramerateLimit(30);

    // Load Map from File
    string line;
    while (getline(map_file, line))
    {
        // Ignore Short Lines
        if (line.length() < 3) continue;

        // Get Command
        char cmd = line[0];
        line.erase(0, 2);

        // Create String Stream
        istringstream line_stream;
        line_stream.str(line);
        float x, y, x2, y2;
        string name;

        // Do Things Depending on Command
        switch(cmd)
        {
            case '#': // Comment
                break;
            case 'o': // Line Obstacle
                line_stream >> x >> y >> x2 >> y2;
                addWall(sf::Vector2f(x, y), sf::Vector2f(x2, y2));
                break;
            case 'b': // Box Obstacle
                line_stream >> x >> y >> x2 >> y2;
                addWall(sf::Vector2f(x, y), sf::Vector2f(x2, y));
                addWall(sf::Vector2f(x2, y), sf::Vector2f(x2, y2));
                addWall(sf::Vector2f(x, y), sf::Vector2f(x, y2));
                addWall(sf::Vector2f(x, y2), sf::Vector2f(x2, y2));
                break;
            case 'n': // Normal Node
                line_stream >> x >> y;
                addNode(sf::Vector2f(x, y));
                break;
            case 'N': // Normal Named Node
                line_stream >> x >> y >> name;
                addNode(sf::Vector2f(x, y), name);
                break;
            case 's': // Start Node
                line_stream >> x >> y;
                start_node = addNode(sf::Vector2f(x, y));
                break;
            case 'e': // End Node
                line_stream >> x >> y;
                end_node = addNode(sf::Vector2f(x, y));
                break;
            default: // Unknown Command
                cout << "UNKNOWN CMD: " << cmd << " (" << line << ")" << endl;
        }
    }
    map_file.close();

    // Debug Mode Indicator
    sf::Text debug_indicator;
    debug_indicator.setFont(ARIAL);
    debug_indicator.setString("DEBUG UI");
    debug_indicator.setCharacterSize(24);
    debug_indicator.setFillColor(sf::Color::Yellow);
    debug_indicator.setPosition(window.getSize().x - 140, 10);
    debug_indicator.setStyle(0 | sf::Text::Bold);

    // Variables
    sf::Clock clock;
    double path_time = 0;
    MapNode* nearest_node = nullptr;
    bool shift_down = false;

    // Loop Until Program Ends
    while (window.isOpen())
    {
        // Set Background Based on UI Mode
        window.clear(sf::Color::Black);
        if (DEBUG_UI)
        {
            window.draw(debug_indicator);
        }
        else
        {
            window.draw(background);
        }

        // Display Map
        display_map(window);

        // Draw Start Node
        if (start_node != nullptr)
        {
            sf::CircleShape start_circle = sf::CircleShape();
            start_circle.setRadius(8);
            start_circle.setPosition(start_node->pos - sf::Vector2f(8, 8));
            start_circle.setFillColor(sf::Color::Red);
            window.draw(start_circle);
        }

        // Draw End Node
        if (end_node != nullptr)
        {
            sf::CircleShape end_circle = sf::CircleShape();
            end_circle.setRadius(8);
            end_circle.setPosition(end_node->pos - sf::Vector2f(8, 8));
            end_circle.setFillColor(sf::Color::Green);
            window.draw(end_circle);
        }

        // Event Listening
        sf::Event event{};
        bool set_start = false, set_end = false;
        while (window.pollEvent(event))
        {
            // Event Listeners
            switch (event.type)
            {
                case sf::Event::Closed:
                    // Close Window
                    window.close();
                    break;
                case sf::Event::KeyPressed:
                    // Reset
                    if (event.key.code == sf::Keyboard::Escape)
                    {
                        reset();
                        DEBUG_UI = false;
                    }

                    // Update Shift State Variable
                    else if (event.key.code == sf::Keyboard::LShift)
                    {
                        shift_down = true;
                    }
                    break;
                case sf::Event::KeyReleased:
                    // Update Shift State Variable
                    if (event.key.code == sf::Keyboard::LShift)
                    {
                        shift_down = false;
                    }

                    // Toggle Debug Mode
                    else if (event.key.code == sf::Keyboard::RShift)
                    {
                        DEBUG_UI = !DEBUG_UI;
                    }
                    break;
                case sf::Event::MouseButtonPressed:
                    // Set Start/End Node
                    if (event.mouseButton.button == sf::Mouse::Left)
                    {
                        if (shift_down)
                        {
                            set_start = true;
                        }
                        else
                        {
                            set_end = true;
                        }
                    }
                    break;
                case sf::Event::MouseMoved:
                    // Find the Nearest Node to Mouse Cursor
                    float min_dist = 100000; // Arbitrary Large Number
                    list<MapNode>::iterator map_iterator;
                    for (map_iterator = school_graph.begin(); map_iterator != school_graph.end(); ++map_iterator)
                    {
                        if (map_iterator->name != " ")
                        {
                            float dist = hypot(map_iterator->pos.x - event.mouseMove.x, map_iterator->pos.y - event.mouseMove.y);
                            if (dist < min_dist)
                            {
                                min_dist = dist;
                                nearest_node = &*map_iterator;
                            }
                        }
                    }

                    // Ignore if Nearest Node is too Far
                    if (min_dist > 20)
                    {
                        nearest_node = nullptr;
                    }
                    break;
            }
        }

        // Display Path if it Exists
        if (end_node != nullptr && end_node->previous != nullptr)
        {
            // Draw Path
            double path_length = 0;
            MapNode* curr = end_node;
            while (curr != start_node)
            {
                draw_line(window, curr->pos, curr->previous->pos, 7, sf::Color(154, 154, 255));
                path_length += distance(*curr, *(curr->previous));
                curr = curr->previous;
            }

            // Create String Stream for Path Information
            ostringstream path_text;
            path_text << "Path Found In: " << fixed << setprecision(3) << path_time << " ms\n";
            path_text << "Path Length: " << fixed << setprecision(1) << path_length * 0.6 << " ft";

            // Path Information Text
            sf::Text text;
            text.setFont(ARIAL);
            text.setString(path_text.str());
            text.setCharacterSize(24);
            text.setFillColor(sf::Color::White);
            window.draw(text);
        }

        // Display Node Connections if Debug Mode
        else if (DEBUG_UI)
        {
            display_graph(window);
        }

        // Deal With Mouse Hover Node
        if (nearest_node != nullptr)
        {
            // Make Node Circle Larger
            sf::CircleShape node_circle = sf::CircleShape();
            node_circle.setRadius(10);
            node_circle.setPosition(nearest_node->pos - sf::Vector2f(10, 10));
            node_circle.setFillColor(sf::Color(100, 100, 100));
            window.draw(node_circle);

            // Set Node to Start Node
            if (set_start)
            {
                reset();
                start_node = nearest_node;
            }

            // Set Node to End Node
            if (set_end)
            {
                reset();
                end_node = nearest_node;
            }

            // Find Path from Start Node to End Node
            if (set_start || set_end)
            {
                clock.restart();
                find_path_dijkstra();
                path_time = clock.getElapsedTime().asMicroseconds() / 1000.0;
            }
        }

        // Update Window Display
        window.display();
    }

    return 0;
}
