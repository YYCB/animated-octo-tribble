// @lcpr-before-debug-begin




// @lcpr-before-debug-end

/*
 * @lc app=leetcode.cn id=344 lang=cpp
 * @lcpr version=30121
 *
 * [344] 反转字符串
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
    void reverseString(vector<char>& s) {
        auto f_pt = s.begin();
        auto sec_pt = s.end()-1;

        while(sec_pt - f_pt >= 1){
            char tmp = *f_pt;
            *f_pt = *sec_pt;
            *sec_pt = tmp;
            f_pt++;
            sec_pt--;
        }        

    }
};
// @lc code=end



/*
// @lcpr case=start
// ["h","e","l","l","o"]\n
// @lcpr case=end

// @lcpr case=start
// ["H","a","n","n","a","h"]\n
// @lcpr case=end

 */

