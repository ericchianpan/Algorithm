#include "graphsp.h"
#include "BinaryHeap.h"

graphsp::graphsp() {;}
graphsp::~graphsp() {;}
void graphsp::Initialization(short hori_tile, short verti_tile, short layer)
{
    _hori_tile_no = hori_tile;
    _verti_tile_no = verti_tile;
    _layer_no = layer;
    _num_vertex = hori_tile * verti_tile * layer;
    _adj_list.resize(_num_vertex);
    _predecessor.resize(_num_vertex);
    _distance.resize(_num_vertex);
}
void graphsp::AddEdge(int from, int to, int weight)
{
    _adj_list[from].push_back(std::make_pair(to, weight));
}
void graphsp::UpdateWeight(int from, int to, int weight)
{
    std::vector<std::list<std::pair<int, int>>> _adj_list;
    for(std::list<std::pair<int, int>>::iterator it = _adj_list[from].begin(); it != _adj_list[from].end(); ++it) {
        if(it->first == to) {
            it->second = weight;
            return;
        }
    }
}
void graphsp::Relax(int from, int to, int weight)
{
    if(_distance[to] > _distance[from] + weight) {
        _distance[to] = _distance[from] + weight;
        _predecessor[to] = from;
    }
}
void graphsp::InitializeSingleSource(int start)
{
    for(int i = 0; i < _num_vertex; ++i) {
        _predecessor[i] = -1;
        _distance[i] = max_distance;
    }
    _distance[start] = 0;
}
void graphsp::Dijkstra(short s_x, short s_y, short s_z, short t_x, short t_y, short t_z)
{
    int start = ToIndex(s_x, s_y, s_z);
    int target = ToIndex(t_x, t_y, t_z); 

    InitializeSingleSource(start);

    BinaryHeap minQueue(_num_vertex);
    minQueue.BuildMinHeap(_distance);

    while(!minQueue.IsEmpty()) {
        int from = minQueue.ExtractMin();
        if(from == target) break;
        if(_distance[from] >= max_distance) break;
        for(std::list<std::pair<int, int>>::iterator it = _adj_list[from].begin(); it != _adj_list[from].end(); ++it) {
            int to = it->first;
            int weight = it->second;
            Relax(from, to, weight);
            minQueue.DecreaseKey(to, _distance[to]);
        }
    }
}
void graphsp::PrintDistance()
{
    std::cout << "print distance" << std::endl;
    for(unsigned i = 0; i < _distance.size(); ++i) {
        std::cout << i << " ";
        std::cout << _distance[i] << std::endl;
    }
}
void graphsp::PrintPredecessor()
{
    std::cout << "print predecessor" << std::endl;
    for(unsigned i = 0; i < _distance.size(); ++i) {
        std::cout << i << " ";
        std::cout << _predecessor[i] << std::endl;
    }
}
void graphsp::GetShortestPath(short t_x, short t_y, short t_z)
{
    shortest_path.clear();
    int target = ToIndex(t_x, t_y, t_z);
    int predecessor = _predecessor[target];
    if(predecessor == -1) {
        std::cout << "no solution" << std::endl;
       return;
    }
    shortest_path.push_front(ToTile(target));
    while(predecessor != -1) {
        shortest_path.push_front(ToTile(predecessor));
        predecessor = _predecessor[predecessor];
    }
}
void graphsp::Reset()
{
    for(unsigned i = 0; i < _adj_list.size(); ++i) {
        _adj_list[i].clear();
    }
}
void graphsp::PrintAdjList()
{
    for(unsigned i = 0; i < _adj_list.size(); ++i) {
        std::cout << "index: " << i << " :" << std::endl;
        PrintAdjList(i);
    }
}
void graphsp::PrintAdjList(int idx)
{
    for(std::list<std::pair<int, int>>::iterator it = _adj_list[idx].begin(); it != _adj_list[idx].end(); ++it) {
        std::cout << "adjacent index: " << it->first << ", ";
        std::cout << "weight: " << it->second << std::endl;
    }
}
inline int graphsp::ToIndex(int x, int y, int z)
{
    return y * (_verti_tile_no) + x + z * (_verti_tile_no * _hori_tile_no);
}
inline std::tuple<int, int, int> graphsp::ToTile(int idx)
{
    short x = (idx % (_verti_tile_no * _hori_tile_no)) % _verti_tile_no;
    short y = (idx % (_verti_tile_no * _hori_tile_no)) / _verti_tile_no;
    short z = idx / (_verti_tile_no * _hori_tile_no);
    return std::make_tuple(x, y, z);
}
void graphsp::BuildRoutingGrid(short s_x, short s_y, short s_z, short t_x, short t_y, short t_z, short layer)
{
    Reset();

    short min_x = s_x < t_x ? s_x : t_x;
    short max_x = s_x > t_x ? s_x : t_x;
    short min_y = s_y < t_y ? s_y : t_y;
    short max_y = s_y > t_y ? s_y : t_y;

    for(short x = min_x; x <= max_x; ++x) {
        for(short y = min_y; y <= max_y; ++y) {
            for(short z = 0; z < layer; ++z) {
                if(z == 0) {
                    if(x != max_x) AddEdge(ToIndex(x, y, z), ToIndex(x + 1, y, z), 1);
                    if(x != min_x) AddEdge(ToIndex(x, y, z), ToIndex(x - 1, y, z), 1);
                    AddEdge(ToIndex(x, y, z), ToIndex(x, y, z + 1), 1);
                }
                else {
                    if(y != max_y) AddEdge(ToIndex(x, y, z), ToIndex(x, y + 1, z), 1);
                    if(y != min_y) AddEdge(ToIndex(x, y, z), ToIndex(x, y - 1, z), 1);
                    AddEdge(ToIndex(x, y, z), ToIndex(x, y, z - 1), 1);
                }
            }
        }
    }
}

