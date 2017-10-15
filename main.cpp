//
//  main.cpp
//  Carpe Diem
//  Created by Tarun Sudhams on 14/10/17.
//  Copyright © 2017 Tarun Sudhams. All rights reserved.
//

#include <iostream>
#include <list>
#include <queue>
#include <assert.h>

using namespace std;
typedef pair<int, int> Nodes;

class Graph
{
    int V;
    list< pair<int, int> > *adj;
public:
    Graph (int V);
    void addEdge(int, int, int); // integer id of the two vertices and the weight of the edge
    vector <int> shortestPath(int);
    bool isReachable(int,int);  // checks if the two nodes are well connected
};

class AdjacencyMatrix
{
private:
    int n;
    int **adj;
    bool *visited;
    
public:
    AdjacencyMatrix(int n);
    void add_edge(int,int,int);
    void floyds();
    void floyd_result(int, int);
};

Graph::Graph(int V) // constructor
{
    this->V = V;
    adj = new list<Nodes> [V];
}


void Graph::addEdge(int u, int v , int weight)
{
    adj[u].push_back(make_pair(v, weight));
    adj[v].push_back(make_pair(u, weight));
}


vector <int> Graph::shortestPath(int u)
{
    priority_queue< Nodes, vector <Nodes> , greater<Nodes> > pq;
    
    vector <int> distance(V, INT_MAX); // find the value for positive infinity in C++ and use that instead of INT_MAX
    
    pq.push(make_pair(0, u));
    
    distance[u] = 0;
    
    while(!pq.empty())
    {
        int k = pq.top().second;
        pq.pop();
        
        list<pair<int,int> >::iterator i;
        
        for(i = adj[k].begin(); i != adj[k].end(); i++)
        {
            int v = (*i).first;
            int weight = (*i).second;
            
            if(distance[v] > distance[k] + weight)
            {
                distance[v] = distance[k] +weight;
                pq.push(make_pair(distance[v], v));
            }
        }
    }
    
    return distance;
}

bool Graph::isReachable(int source, int destination)
{
    
    if (source == destination)
        return true;
    
    
    bool *visited = new bool[V];
    for (int i = 0; i < V; i++)
    {
        visited[i] = false;
    }
    
    list<int> queue;
    
    visited[source] = true;
    queue.push_back(source);
    
    
    list<pair<int,int> >::iterator i;
    
    while (!queue.empty())
    {
        
        source = queue.front();
        queue.pop_front();
        
        for (i = adj[source].begin(); i != adj[source].end(); i++)
        {
            
            int v = (*i).first;
            
            if (v == destination)
                return true;
            
            if (!visited[v])
            {
                visited[v] = true;
                queue.push_back(v);
            }
        }
    }
    
    return false;
}

AdjacencyMatrix::AdjacencyMatrix(int n)
{
    this->n = n;
    visited = new bool[n];
    adj = new int* [n];
    
    for(int i = 0; i < n; i++)
    {
        adj[i] = new int[n];
        
        for(int j = 0; j < n; j++)
        {
            adj[i][j] = 0;
        }
    }
}

void AdjacencyMatrix::add_edge(int a, int b, int weight)
{
    adj[a][b] = weight;
    adj[b][a] = weight;
}

void AdjacencyMatrix::floyds()
{
    
    for (int k = 0; k < n; k++)
    {
        for (int i = 0; i < n; i++)
        {
            for (int j = 0;  j < n; j++)
            {
                if((adj[i][k] * adj[k][j] != 0) && (i != j))
                {
                    if ((adj[i][k] + adj[k][j] < adj[i][j]) || (adj[i][j] == 0))
                    {
                        adj[i][j] = adj[i][k] + adj[k][j];
                    }
                }
                
            }
        }
    }
}

void AdjacencyMatrix::floyd_result(int source, int destination)
{
    cout << adj[source][destination] << endl;
}


void BusFair()
{
    int n, m;
    
    cin >> n >> m;
    
    Graph g(n);
    for(int i = 0; i < m; i++)
    {
        int a, b, w;
        cin >> a >> b >> w;
        g.addEdge(a, b, w);
    }
    
    int u;
    cin >> u; // u denotes Mike's location on the map
    
    int l; // l denotes the number of cars on the map
    
    cin >> l;
    
    vector <int> array;
    
    int input;
    
    for(int i = 0; i < l; i++)
    {
        cin >> input;
        array.push_back(input);
    }
    
    vector <int> shortest_path = g.shortestPath(u);
    
    for(int i = 0; i < shortest_path.size(); i++)
    {
        cout << "The elements present in shortest path: " << shortest_path[i] << endl;
    }
    
    int min_pos = distance(shortest_path.begin(),min_element(shortest_path.begin(),shortest_path.end()));
    
    bool state = false;
    
    while(state != true)
    {
        for (int i = 0; i < array.size(); ++i)
        {
            if(array[i] == min_pos)
            {
                state = true;
                break;
            }
        }
        
        if(state == false)
        {
            shortest_path.erase(std::remove(shortest_path.begin(), shortest_path.end(), min_pos), shortest_path.end());
            min_pos = distance(shortest_path.begin(),min_element(shortest_path.begin(),shortest_path.end()));
        }
    }
    
    
}




int main()
{
    string section;
    cin >> section;
    
    if(section == "NearestDriver")
        BusFair();

    return 0;
}
