#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <queue>
#include <set>
#include <cstdlib>
#include <chrono>
#include "Game.h"
#include "Blocks/Blocks.h"

using std::vector;
using std::string;
using std::shared_ptr;
using std::make_shared;
using std::cout;
using std::endl;
using std::queue;
using std::priority_queue;
using std::set;
using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

struct Info
{
    int cost, index;
    bool operator< (const Info &a) const
    {
        return cost < a.cost;
    }
};
priority_queue<Info> q;
vector<shared_ptr<Game>> games;
set<string> vis;

void gameInit()
{
    auto game_ptr = make_shared<Game>(Game(5, 4, 0));
    game_ptr->blocks.push_back(make_shared<Block_2x2>(Block_2x2(game_ptr, 0, 1)));
    game_ptr->blocks.push_back(make_shared<Block_2x1>(Block_2x1(game_ptr, 0, 0)));
    game_ptr->blocks.push_back(make_shared<Block_2x1>(Block_2x1(game_ptr, 2, 0)));
    game_ptr->blocks.push_back(make_shared<Block_2x1>(Block_2x1(game_ptr, 0, 3)));
    game_ptr->blocks.push_back(make_shared<Block_2x1>(Block_2x1(game_ptr, 2, 3)));
    game_ptr->blocks.push_back(make_shared<Block_1x2>(Block_1x2(game_ptr, 2, 1)));
    game_ptr->blocks.push_back(make_shared<Block_1x1>(Block_1x1(game_ptr, 4, 0)));
    game_ptr->blocks.push_back(make_shared<Block_1x1>(Block_1x1(game_ptr, 4, 3)));
    game_ptr->blocks.push_back(make_shared<Block_1x1>(Block_1x1(game_ptr, 3, 1)));
    game_ptr->blocks.push_back(make_shared<Block_1x1>(Block_1x1(game_ptr, 3, 2)));
    game_ptr->calcState();
    q.push({game_ptr->estimate(), 0});
    games.push_back(game_ptr);
    vis.insert(game_ptr->toString());
}

int main()
{
    auto start = steady_clock::now();
    freopen("huarong.txt", "w", stdout);
    gameInit();
    while (!q.empty())
    {
        Info info = q.top();
        q.pop();
        auto game = games[info.index];
        for (int i = 0; i < game->blocks.size(); i++)
        {
            for (Dir dir = UP; dir <= RIGHT; dir = (Dir)(dir + 1))
            {
                if (game->blocks[i]->movable(dir))
                {
                    auto newGame = game->copy();
                    newGame->blocks[i]->move(dir);
                    newGame->lastIndex = info.index;
                    string newStr = newGame->toString();
                    if (vis.find(newStr) != vis.end()) continue;
                    Info newInfo{newGame->estimate(), games.size()};
                    q.push(newInfo);
                    games.push_back(newGame);
                    vis.insert(newStr);
                    if (newGame->blocks[0]->estimate() == 0)
                    {
                        cout << "Finish! Steps: " << newGame->moved << endl;
                        int index = newInfo.index;
                        while (index != 0)
                        {
                            games[index]->printState();
                            index = games[index]->lastIndex;
                        }
                        auto end = steady_clock::now();
                        auto duration = duration_cast<milliseconds>(end - start).count();
                        cout << "Time: " << duration << endl;
                        return 0;
                    }
                }
            }
        }
    }
    cout << "No solution" << endl;
    return 0;
}