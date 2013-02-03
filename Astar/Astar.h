#pragma once
#ifndef __Astar_h_
#define __Astar_h_

#include <vector>
#include <map>

using namespace std;


class Cell
{
public:
    Cell(int cost);
    bool isWalkable();
    int getFullCost() const;
    void setFullCost(int cost);
    int getCost() const;
    void setHeuristic(int value);
    int getEstimate() const;
    
    void setParent(Cell* parent);
    Cell* getParent();
    
    void setCoords(int x, int y);
    pair<int, int> getCoords() const;
    
    int value;
    
    bool closed = false;
    bool open = false;
private:
    Cell* mParent = 0;
    
    int mCost; // cost for going through this cell, 0 for non-crossable cell
    int estimate; // estimated distance + cost
    int fullCost; // cost from start to this cell
    int heuristic; // estimated distance to goal
    
    pair<int, int> mCoords;
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
    
    vector<Cell*> getSurroundings(Cell*);
    
    vector<pair<int, int> > getPath();
    
    void setEmptyValue(int value);
    
    
protected:
    bool isPositionValid(pair<int, int>) const;
    bool isClosed(pair<int, int>) const;
    bool isFreeCell(pair<int, int>) const;
    bool isGridValid() const;
    bool isFinalNode(Cell* cell) const;
    
    void buildFinalPath(Cell* finalCell);
    
    int getCellHeuristic(Cell* cell);
    
    vector<vector<Cell*> > mGrid; // working grid
    vector<pair<int, int> > mDirections; // directions available from a given point
    multimap<int, Cell*> mOpenList;
    
    vector<pair<int, int> > mPath; // path already processed
    
    pair<int, int> mStart;
    pair<int, int> mEnd;
    
    int empty; // value for authorized cells
};

#endif