/*
 * @lc app=leetcode.cn id=2697 lang=cpp
 * @lcpr version=30121
 *
 * [2697] 字典序最小回文串
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
    string makeSmallestPalindrome(string s) {
        auto f_p = s.begin();
        auto sec_p = s.end()-1;

        while(sec_p - f_p >= 1){
            if(*f_p != *sec_p){
                if(*f_p > *sec_p)
                    *f_p = *sec_p;
                else 
                    *sec_p = *f_p;
            }

            f_p++;
            sec_p--;
        }

        return s;
    }
};
// @lc code=end



/*
// @lcpr case=start
// "egcfe"\n
// @lcpr case=end

// @lcpr case=start
// "abcd"\n
// @lcpr case=end

// @lcpr case=start
// "seven"\n
// @lcpr case=end

 */

