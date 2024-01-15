#include "Game.h"

Game::Game(int _row, int _col, int _moved)
{
    row = _row;
    col = _col;
    moved = _moved;
    map = new char*[row];
    for (int i = 0; i < row; i++)
    {
        map[i] = new char[col + 1];
        map[i][col] = '\0';
    }
}

void Game::clearMap()
{
    for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
            map[i][j] = '0';
}

void Game::calcState()
{
    clearMap();
    for (int i = 0; i < blocks.size(); i++) blocks[i]->fill();
}

void Game::printState()
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++) putchar(map[i][j]);
        putchar('\n');
    }
    putchar('\n');
}

string Game::toString()
{
    string s;
    for (int i = 0; i < row; i++) s += map[i];
    return s;
}

shared_ptr<Game> Game::copy()
{
    auto gp = make_shared<Game>(Game(row, col, moved));
    for (int i = 0; i < blocks.size(); i++)
        gp->blocks.push_back(blocks[i]->copy(gp));
    gp->calcState();
    return gp;
}

int Game::estimate()
{
    int t = moved;
    for (int i = 0; i < blocks.size(); i++) t += blocks[i]->estimate();
    return t;
}