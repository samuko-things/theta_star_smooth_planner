#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

// ======================================================
// GRAPH NODE (PYTHON-EQUIVALENT)
// ======================================================
struct GraphNode
{
    int x, y;

    double cost = 0;
    double heuristic = 0;

    GraphNode* prev = nullptr;

    GraphNode(int _x = 0, int _y = 0)
        : x(_x), y(_y) {}
};

// ======================================================
// PRIORITY QUEUE
// ======================================================
struct CompareNode
{
    bool operator()(GraphNode* a, GraphNode* b)
    {
        return (a->cost + a->heuristic) > (b->cost + b->heuristic);
    }
};

// ======================================================
// THETA STAR NODE
// ======================================================
class ThetaStarPlanner : public rclcpp::Node
{
public:
    ThetaStarPlanner()
        : Node("theta_star_planner"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        this->declare_parameter("cost_limit", 20);
        cost_limit_ = this->get_parameter("cost_limit").as_int();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10,
            std::bind(&ThetaStarPlanner::mapCallback, this, _1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&ThetaStarPlanner::goalCallback, this, _1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/theta_star/path", 10);

        RCLCPP_INFO(this->get_logger(), "Theta* (GraphNode version) ready");
    }

private:
    // ======================================================
    // VARIABLES
    // ======================================================
    nav_msgs::msg::OccupancyGrid map_;
    double resolution_;
    int cost_limit_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ======================================================
    // MAP CALLBACK
    // ======================================================
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        resolution_ = msg->info.resolution;
    }

    // ======================================================
    // GOAL CALLBACK
    // ======================================================
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
    {
        if (map_.data.empty())
            return;

        geometry_msgs::msg::Pose start;

        try
        {
            auto tf = tf_buffer_.lookupTransform(
                map_.header.frame_id, "base_link", tf2::TimePointZero);

            start.position.x = tf.transform.translation.x;
            start.position.y = tf.transform.translation.y;
            start.orientation = tf.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        auto path = plan(start, goal->pose);
        path_pub_->publish(fillUpPath(path));
    }

    // ======================================================
    // PLAN (THETA*)
    // ======================================================
    nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose &start,
                              const geometry_msgs::msg::Pose &goal)
    {
        std::priority_queue<GraphNode*, std::vector<GraphNode*>, CompareNode> open;
        std::unordered_set<long long> closed;
        std::unordered_map<long long, double> g_score;

        auto start_node = new GraphNode(worldToGrid(start));
        auto goal_node  = new GraphNode(worldToGrid(goal));

        start_node->cost = 0;
        start_node->heuristic = heuristic(*start_node, *goal_node);
        start_node->prev = start_node;

        open.push(start_node);
        g_score[key(start_node->x, start_node->y)] = 0;

        std::vector<std::pair<int,int>> dirs = {
            {-1,0},{1,0},{0,-1},{0,1},
            {-1,-1},{-1,1},{1,-1},{1,1}
        };

        GraphNode* final = nullptr;
        bool found = false;

        while (!open.empty())
        {
            GraphNode* current = open.top();
            open.pop();

            if (closed.count(key(current->x, current->y)))
                continue;

            if (current->x == goal_node->x &&
                current->y == goal_node->y)
            {
                final = current;
                found = true;
                break;
            }

            closed.insert(key(current->x, current->y));

            for (auto d : dirs)
            {
                int nx = current->x + d.first;
                int ny = current->y + d.second;

                if (!inBounds(nx, ny))
                    continue;

                if (map_.data[toIndex(nx, ny)] >= cost_limit_)
                    continue;

                auto neighbor = new GraphNode(nx, ny);

                GraphNode* parent = current;

                if (current->prev &&
                    lineOfSight(current->prev->x, current->prev->y, nx, ny))
                {
                    parent = current->prev;
                }

                double new_cost = getG(parent) +
                                  heuristic(*parent, *neighbor);

                long long k = key(nx, ny);

                if (!g_score.count(k) || new_cost < g_score[k])
                {
                    g_score[k] = new_cost;

                    neighbor->cost = new_cost;
                    neighbor->heuristic = heuristic(*neighbor, *goal_node);
                    neighbor->prev = parent;

                    open.push(neighbor);
                }
            }
        }

        nav_msgs::msg::Path path;
        path.header.frame_id = map_.header.frame_id;

        if (!found)
            return path;

        GraphNode* node = final;

        while (node)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = map_.header.frame_id;
            ps.pose = gridToWorld(*node);

            path.poses.push_back(ps);

            if (node->prev == node)
                break;

            node = node->prev;
        }

        std::reverse(path.poses.begin(), path.poses.end());
        return path;
    }

    // ======================================================
    // HELPERS
    // ======================================================
    double heuristic(GraphNode a, GraphNode b)
    {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    double getG(GraphNode* n)
    {
        return n ? n->cost : 0;
    }

    long long key(int x, int y)
    {
        return ((long long)x << 32) | y;
    }

    bool inBounds(int x, int y)
    {
        return x >= 0 && y >= 0 &&
               x < (int)map_.info.width &&
               y < (int)map_.info.height;
    }

    int toIndex(int x, int y)
    {
        return y * map_.info.width + x;
    }

    GraphNode worldToGrid(const geometry_msgs::msg::Pose &p)
    {
        int x = (p.position.x - map_.info.origin.position.x) / resolution_;
        int y = (p.position.y - map_.info.origin.position.y) / resolution_;
        return GraphNode(x, y);
    }

    geometry_msgs::msg::Pose gridToWorld(const GraphNode &n)
    {
        geometry_msgs::msg::Pose p;
        p.position.x = n.x * resolution_ + map_.info.origin.position.x;
        p.position.y = n.y * resolution_ + map_.info.origin.position.y;
        return p;
    }

    // ======================================================
    // LINE OF SIGHT
    // ======================================================
    bool lineOfSight(int x0, int y0, int x1, int y1)
    {
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (map_.data[toIndex(x0, y0)] >= cost_limit_)
                return false;

            if (x0 == x1 && y0 == y1)
                break;

            int e2 = 2 * err;

            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx)  { err += dx; y0 += sy; }
        }

        return true;
    }

    // ======================================================
    // ADD STRAIGHT LINE (PYTHON MATCH)
    // ======================================================
    std::vector<geometry_msgs::msg::PoseStamped>
    addStraightLinePoses(const geometry_msgs::msg::PoseStamped &a,
                         const geometry_msgs::msg::PoseStamped &b,
                         double res)
    {
        std::vector<geometry_msgs::msg::PoseStamped> out;

        double dx = b.pose.position.x - a.pose.position.x;
        double dy = b.pose.position.y - a.pose.position.y;

        double dist = std::hypot(dx, dy);
        if (dist == 0) return out;

        int steps = (int)(dist / res);
        if (steps <= 0) return out;

        double ix = dx / steps;
        double iy = dy / steps;

        for (int i = 0; i < steps; i++)
        {
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = a.header.frame_id;
            p.pose.position.x = a.pose.position.x + ix * i;
            p.pose.position.y = a.pose.position.y + iy * i;
            p.pose.orientation = a.pose.orientation;
            out.push_back(p);
        }

        out.push_back(b);
        return out;
    }

    // ======================================================
    // FILL UP PATH (PYTHON MATCH)
    // ======================================================
    nav_msgs::msg::Path fillUpPath(const nav_msgs::msg::Path &path)
    {
        nav_msgs::msg::Path filled;
        filled.header.frame_id = path.header.frame_id;

        if (path.poses.empty())
            return filled;

        filled.poses.push_back(path.poses[0]);

        for (size_t i = 1; i < path.poses.size(); i++)
        {
            auto seg = addStraightLinePoses(
                path.poses[i - 1],
                path.poses[i],
                resolution_);

            for (size_t j = 1; j < seg.size(); j++)
                filled.poses.push_back(seg[j]);
        }

        return filled;
    }
};

// ======================================================
// MAIN
// ======================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThetaStarPlanner>());
    rclcpp::shutdown();
    return 0;
}