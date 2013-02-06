
#include "Astar.h"

using namespace std;

Cell::Cell(int cost) : open(false), closed(false), mParent(0), mCost(cost)
{
}

bool Cell::isWalkable()
{
    return !closed && mCost > 0;
}

void Cell::setFullCost(int cost)
{
    fullCost = cost;
    estimate = heuristic + fullCost;
}

int Cell::getFullCost() const
{
    return fullCost;
}

int Cell::getCost() const
{
    return mCost;
}

void Cell::setCoords(int x, int y)
{
    mCoords.first = x;
    mCoords.second = y;
}

pair<int, int> Cell::getCoords() const
{
    return mCoords;
}

void Cell::setHeuristic(int value)
{
    heuristic = value;
}

int Cell::getEstimate() const
{
    return estimate;
}

void Cell::setParent(Cell* parent)
{
    mParent = parent;
}

Cell* Cell::getParent()
{
    return mParent;
}

Astar::Astar() : empty(0), mStart(0, 0), mEnd(0, 0)
{
}

bool Astar::run()
{
    if (!isGridValid() || !isPositionValid(mStart) || !isPositionValid(mEnd) || !isFreeCell(mStart) || !isFreeCell(mEnd)) {
        return false;
    }
    
    mPath.resize(0);
    mGrid[mStart.first][mStart.second]->open = true;
    mOpenList.insert(pair<int, Cell*>(0, mGrid[mStart.first][mStart.second]));
    
    while (true) {
        
        if (mOpenList.empty()) {
            return false; // no path available
        }
        
        // get the node with best estimated value
        multimap<int, Cell*>::iterator front = mOpenList.begin();
        Cell* currentNode = (*front).second;
        mOpenList.erase(front);
        currentNode->closed = true;
        currentNode->open = false;
        
        if (isFinalNode(currentNode)) {
            buildFinalPath(currentNode);
            break;
        }
        
        // browse and process surrounding nodes
        vector<Cell*> nodes = getSurroundings(currentNode);
        vector<Cell*>::iterator it;
        for (it = nodes.begin(); it < nodes.end(); it++) {
            Cell* node = (*it);
            if (node->isWalkable()) {
                if (!node->open) { // add to open list
                    node->open = true;
                    node->setParent(currentNode);
                    node->setFullCost(currentNode->getCost() + node->getCost());
                    node->setHeuristic(getCellHeuristic(node));
                    mOpenList.insert(pair<int, Cell*>(node->getEstimate(), node));
                } else if (node->getFullCost() > currentNode->getCost() + node->getCost()) { // recalculate path to node
                    node->setParent(currentNode);
                    node->setFullCost(currentNode->getCost() + node->getCost());
                }
            }
        }
        
        // reorder the open list
        multimap<int, Cell*>::iterator mapIt;
        for (mapIt = mOpenList.begin(); mapIt != mOpenList.end(); mapIt++) {
            Cell* node = (*mapIt).second;
            if (node->getFullCost() != (*mapIt).first) {
                mOpenList.insert(pair<int, Cell*>(node->getEstimate(), node));
                mapIt = mOpenList.erase(mapIt); // requires c++11 support
            }
        }
    }
    return true;
}

void Astar::setStartPoint(std::pair<int, int> point)
{
    mStart = point;
}

void Astar::setEndPoint(std::pair<int, int> point)
{
    mEnd = point;
}

void Astar::setDirections(vector<pair<int, int> > directions)
{
    mDirections = directions;
}

void Astar::setGrid(vector<vector<int> > grid)
{
    mGrid.clear();
    mGrid.resize(grid.size());
    int cost = 0;
    for (int x = 0; x < grid.size(); x++) {
        mGrid[x].clear();
        for (int y = 0; y < grid[x].size(); y++) {
            if (grid[x][y] == empty) {
                cost = 10;
            } else {
                cost = 0;
            }
            Cell* cell = new Cell(cost);
            cell->setCoords(x, y);
            cell->value = grid[x][y];
            mGrid[x].push_back(cell);
        }
    }
}

vector<Cell*> Astar::getSurroundings(Cell * cell)
{
    pair<int, int> coords = cell->getCoords();
    int x = coords.first;
    int y = coords.second;
    
    Cell* neighbour;
    vector<Cell*> cells;
    vector<pair<int, int> >::iterator itDir;
    
    for (itDir = mDirections.begin(); itDir < mDirections.end(); itDir++) {
        pair<int, int> pos (x + itDir->first, y + itDir->second);
        if (isPositionValid(pos)) {
            neighbour = mGrid[pos.first][pos.second];
            cells.push_back(neighbour);
        }
    }
    
    return cells;
}

int Astar::getCellHeuristic(Cell* cell)
{
    int x = abs(cell->getCoords().first - mEnd.first);
    int y = abs(cell->getCoords().second - mEnd.second);
    
    return x + y;
}

void Astar::buildFinalPath(Cell* finalCell)
{
    Cell* cell = finalCell;
    vector<pair<int, int> > tmp;
    while (cell) {
        tmp.push_back(cell->getCoords());
        cell = cell->getParent();
    }
    
    // flip the result
    vector<pair<int, int> >::reverse_iterator it;
    for (it = tmp.rbegin(); it < tmp.rend(); it++) {
        mPath.push_back(*it);
    }
}

vector<pair<int, int> > Astar::getPath()
{
    return mPath;
}

bool Astar::isGridValid() const
{
    if (mGrid.empty() || mGrid.front().empty()) {
        return false;
    }
    
    unsigned int expectedSize = mGrid.front().size();
    for (int i = 0; i < mGrid.size(); i++) {
        if (mGrid[i].size() != expectedSize) {
            return false;
        }
    }
    
    return true;
}

bool Astar::isPositionValid(pair<int, int> pos) const
{
    int x = pos.first;
    int y = pos.second;
    
    if (x < 0 || y < 0) {
        return false;
    }
    if (x >= mGrid.size() || y >= mGrid.front().size()) {
        return false;
    }
    return true;
}

void Astar::setEmptyValue(int value)
{
    empty = value;
}

bool Astar::isClosed(pair<int, int> pos) const
{
    return mGrid[pos.first][pos.second]->closed;
}

bool Astar::isFreeCell(pair<int, int> pos) const
{
    return mGrid[pos.first][pos.second]->value == empty;
}

bool Astar::isFinalNode(Cell* cell) const
{
    return cell->getCoords() == mEnd;
}
