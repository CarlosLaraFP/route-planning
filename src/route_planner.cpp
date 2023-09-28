#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    RouteModel::Node& start_node = model.FindClosestNode(start_x, start_y);
    RouteModel::Node& end_node = model.FindClosestNode(end_x, end_y);
    // "this" is a raw pointer that points to the instance of this class
    // start_node and end_node are additional raw pointers (private to this class)
    // we assign these pointers to the memory locations of the node references
    this->start_node = &start_node;
    this->end_node = &end_node;
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // passing a copy of the underlying Node
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // populates the neighbors vector of Node pointers
    current_node->FindNeighbors();

    // auto& avoids unnecessary copying
    for (auto& neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->visited = true;
        // adding visited neighbors to the open list
        this->open_list.emplace_back(neighbor);
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(
        this->open_list.begin(),
        this->open_list.end(),
        [] (const RouteModel::Node* a, const RouteModel::Node* b) {
            return a->g_value + a->h_value > b->g_value + b->h_value;
        }
    );

    // Get the last node because the node with the smallest f value is closest to the goal
    auto closest = this->open_list.back();
    // Since we copied the pointer into a separate variable, we remove the original one from the vector of open node pointers
    this->open_list.pop_back();

    return closest;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    std::vector<RouteModel::Node> path_found;

    while (current_node != nullptr) {
        if (current_node->parent != nullptr) this->distance += current_node->distance(*current_node->parent);
        path_found.emplace_back(*current_node);
        current_node = current_node->parent;
    }

    std::reverse(path_found.begin(), path_found.end());

    this->distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;
    current_node->visited = true;
    open_list.emplace_back(current_node);

    while (current_node != this->end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }

    this->m_Model.path = ConstructFinalPath(current_node);
}
