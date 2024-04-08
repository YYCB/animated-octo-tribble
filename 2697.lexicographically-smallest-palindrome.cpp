/*
 * @lc app=leetcode.cn id=2697 lang=cpp
 * @lcpr version=30121
 *
 * [2697] 字典序最小回文串
 */

// @lc code=start
#include <string>
using string = std::string;
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

