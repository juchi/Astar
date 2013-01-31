#pragma once
#ifndef __Astar_h_
#define __Astar_h_

#include <vector>

using namespace std;


struct Cell
{
    int value;
    bool closed;
};


class Astar
{
public:
    Astar();
    
    bool run();
    
    void setStartPoint(pair<int, int>);
    void setEndPoint(pair<int, int>);
    
    void setDirections(vector<pair<int, int> >);
    void setGrid(vector<vector<int> >);
    
    vector<pair<int, int> > getPath();
    
    void setEmptyValue(int value);
    
    
protected:
    bool isPositionValid(pair<int, int>);
    bool closed(pair<int, int>);
    
    vector<vector<Cell> > mGrid; // working grid
    vector<pair<int, int> > mDirections; // directions available from a given point
    
    vector<pair<int, int> > mPath; // path already processed
    
    pair<int, int> mStart;
    pair<int, int> mEnd;
    
    int empty; // value for authorized cells
};

#endif