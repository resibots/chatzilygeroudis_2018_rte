#ifndef MCTS_ASTAR_A_STAR_HPP
#define MCTS_ASTAR_A_STAR_HPP

#include <vector>
#include <map>
#include <algorithm>

namespace astar {
    struct Node {
        int _x, _y, _n_x, _n_y;

        Node()
            : _x(-1), _y(-1), _n_x(-1), _n_y(-1) {}

        Node(int x, int y, int n_x = -1, int n_y = -1)
            : _x(x), _y(y), _n_x(n_x), _n_y(n_y) {}

        std::vector<Node> neighbours(std::function<bool(int, int, int, int)> colliding)
        {
            std::vector<Node> to_ret;
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (i == 0 && j == 0)
                        continue;
                    int x_new = _x + i;
                    int y_new = _y + j;
                    if (x_new >= 0 && x_new < _n_x && y_new >= 0 && y_new < _n_y && !colliding(_x, _y, x_new, y_new))
                        to_ret.push_back(Node(x_new, y_new, _n_x, _n_y));
                }
            }

            return to_ret;
        }

        bool operator==(const Node& other) const
        {
            return (_x == other._x && _y == other._y);
        }

        bool operator<(const Node& other) const
        {
            if (_x < other._x)
                return true;
            else if (_x > other._x)
                return false;
            else
                return (_y < other._y);
        }

        friend std::ostream& operator<<(std::ostream& os, const Node& n);
    };

    std::ostream& operator<<(std::ostream& os, const Node& n)
    {
        os << "(" << n._x << ", " << n._y << ")";
        return os;
    }

    struct Euclidean {
        double operator()(const Node& s, const Node& e)
        {
            double dx = s._x - e._x;
            double dy = s._y - e._y;
            return std::sqrt(dx * dx + dy * dy);
        }
    };

    struct SimpleCost {
        double operator()(const Node& s, const Node& e)
        {
            return 1.0;
        }
    };

    template <typename Heuristic = Euclidean, typename Cost = SimpleCost>
    struct AStar {
        std::vector<Node> search(const Node& start, const Node& goal, std::function<bool(int, int, int, int)> colliding, int Nx, int Ny)
        {
            std::map<Node, Node> came_from;
            std::vector<Node> closed_set;
            std::vector<Node> open_set;
            open_set.push_back(start);

            std::map<Node, double> g_score, f_score;
            for (int x = 0; x < Nx; x++) {
                for (int y = 0; y < Ny; y++) {
                    Node tmp(x, y, Nx, Ny);
                    g_score[tmp] = std::numeric_limits<double>::max();
                    f_score[tmp] = std::numeric_limits<double>::max();
                }
            }
            g_score[start] = 0.0;
            f_score[start] = Heuristic()(start, goal);

            // std::cout << "Start: " << start << std::endl;
            // std::cout << "Goal: " << goal << std::endl;

            while (open_set.size() > 0) {
                std::sort(open_set.begin(), open_set.end(), [&f_score](const Node& a, const Node& b) { auto it1 = f_score.find(a); auto it2 = f_score.find(b); if(it2 == f_score.end()) return true; if(it1 == f_score.end()) return false; return (f_score[a]<f_score[b]); });
                Node current = open_set[0];
                if (current == goal) {
                    return _path(came_from, current);
                }

                open_set.erase(open_set.begin());
                closed_set.push_back(current);

                for (auto neigh : current.neighbours(colliding)) {
                    if (std::find(closed_set.begin(), closed_set.end(), neigh) != closed_set.end())
                        continue;
                    double tentative_score = g_score[current] + Cost()(current, neigh);
                    if (std::find(open_set.begin(), open_set.end(), neigh) == open_set.end()) {
                        open_set.push_back(neigh);
                    }
                    else if (tentative_score >= g_score[neigh])
                        continue;
                    came_from[neigh] = current;
                    g_score[neigh] = tentative_score;
                    f_score[neigh] = g_score[neigh] + Heuristic()(neigh, goal);
                }
            }

            return std::vector<Node>();
        }

        std::vector<Node> _path(std::map<Node, Node> came_from, Node current)
        {
            std::vector<Node> total_path;
            total_path.push_back(current);
            auto cur = current;
            while (came_from.find(cur) != came_from.end()) {
                cur = came_from[cur];
                total_path.push_back(cur);
            }

            std::reverse(total_path.begin(), total_path.end());
            return total_path;
        }
    };
}

#endif
