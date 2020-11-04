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
    end_node->g_value = 0;
    end_node->h_value = CalculateHValue(end_node);
    start_node->g_value = 0;
    start_node->h_value = CalculateHValue(start_node);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    current_node->visited = true;
    for (auto& neighbor: current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;
        open_list.emplace_back(neighbor);
    }
}

static bool compare(RouteModel::Node* node1, RouteModel::Node* node2) {
    return (node1->g_value + node1->h_value) < (node2->g_value + node2->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), compare);
    RouteModel::Node* lowest = open_list.front();
    open_list.erase(open_list.begin());
    return lowest;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node && current_node != start_node) {
        if (current_node->parent) {
            distance += current_node->distance(*current_node->parent);
        }
        path_found.insert(path_found.begin(), *current_node);
        current_node = current_node->parent;
    }
    path_found.insert(path_found.begin(), *start_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    while (current_node != end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(end_node);
}