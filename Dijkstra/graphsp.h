#ifndef graphsp_h
#define graphsp_h

#include <climits>
#include <vector>
#include <list>
#include <map>

const int max_distance = 1000000;

class graphsp
{
    public:
        graphsp();
        ~graphsp();

        void Initialization(short hori_tile, short verti_tile, short layer);
        void AddEdge(int from, int to, int weight);
        void UpdateWeight(int from, int to, int weight);
        void Relax(int from, int to, int weight);
        void InitializeSingleSource(int start);
        void Dijkstra(short s_x, short s_y, short s_z, short t_x, short t_y, short t_z);
        void PrintDistance();
        void PrintPredecessor();
        void PrintAdjList();
        void PrintAdjList(int idx);
        void GetShortestPath(short t_x, short t_y, short t_z);
        void BuildRoutingGrid(short s_x, short s_y, short s_z, short t_x, short t_y, short t_z, short layer);

        std::list<std::tuple<int, int, int>> shortest_path;

    private:
        int _num_vertex;
        short _hori_tile_no;
        short _verti_tile_no;
        short _layer_no;

        std::vector<std::list<std::pair<int, int>>> _adj_list; // to vertex and weight
        std::vector<int> _predecessor;
        std::vector<int> _distance;

        inline int ToIndex(int x, int y, int z);
        inline std::tuple<int, int, int> ToTile(int idx);
        void Reset();

};

#endif
