/*
 * @lc app=leetcode.cn id=821 lang=cpp
 * @lcpr version=30121
 *
 * [821] 字符的最短距离
 */


// @lcpr-template-start
using namespace std;
#include <algorithm>
#include <array>
#include <bitset>
#include <climits>
#include <deque>
#include <functional>
#include <iostream>
#include <list>
#include <queue>
#include <stack>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
// @lcpr-template-end
// @lc code=start
class Solution {
public:
    vector<int> shortestToChar(string s, char c) {
        
        vector<int> indexs;
        for (int i = 0; i < s.size(); i++){
            if (s.at(i) == c){
                indexs.emplace_back(i);
            }
        }
        
        int f_p{0};
        int sec_p{1};
        vector<int> res;
        res.resize(s.size());
        for (int i = 0; i < s.size(); i++){

            if(i<=indexs.front()){
                res.at(i)=abs(indexs.front()-i);
            }else if(i>=indexs.back()){
                res.at(i)=abs(indexs.back()-i);
            }else{
                if (i>indexs.at(sec_p)){
                    f_p = sec_p;
                    sec_p++;
                }
                
                res.at(i)=
                    std::min(
                        abs( indexs.at(f_p) - i ),
                        abs( indexs.at(sec_p)  - i ));
            }
        }

        return res;
    }
};
// @lc code=end



/*
// @lcpr case=start
// "loveleetcode"\n"e"\n
// @lcpr case=end

// @lcpr case=start
// "aaab"\n"b"\n
// @lcpr case=end

 */

