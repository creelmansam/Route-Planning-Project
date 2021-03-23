#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(model.FindClosestNode(start_x, start_y));
    end_node = &(model.FindClosestNode(end_x, end_y));
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return end_node->distance(*node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();
    for (auto* node : current_node->neighbors)
    {
        if(!node->visited)
        {
            node->parent = current_node;
            node->g_value = current_node->g_value + current_node->distance(*node);
            node->h_value = CalculateHValue(node);
            node->visited = true;

            open_list.push_back(node);
        }
        
    }
    
}

RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(open_list.begin(), open_list.end(), 
                [](const RouteModel::Node* a, const RouteModel::Node* b)
                {
                    return a->g_value + a->h_value > b->g_value + b->h_value;
                });
    RouteModel::Node* next_node = open_list.back();
    open_list.pop_back();

    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.push_back(*current_node);
    RouteModel::Node* node = current_node;

    while (node != start_node)
    {
        distance += node->distance(*(node->parent));
        path_found.emplace_back(*node->parent);
        node = node->parent;
    }
    
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    open_list.push_back(start_node);
    start_node->visited = true;

    while (!open_list.empty())
    {
        current_node = NextNode();

        if(current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            std::cout << "Found a final path!\n";
            return;
        }
        
        // # Continue the Search
        AddNeighbors(current_node);
    }
    
    std::cout << "Failed to find a path to the end\n";
}