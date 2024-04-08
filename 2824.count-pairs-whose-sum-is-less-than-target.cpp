/*
 * @lc app=leetcode.cn id=2824 lang=cpp
 * @lcpr version=30121
 *
 * [2824] 统计和小于目标的下标对数目
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
    int countPairs(vector<int>& nums, int target) {
        auto f_p = nums.begin();
        auto sec_p = nums.begin() + 1;
        int cnt{0};
        while (sec_p != nums.end()){
            auto i = f_p;
            while(i!=sec_p){
                if(*i+*sec_p < target){
                    cnt++;
                }
                i++;
            }
            sec_p++;
        }
        return cnt;
    }
};
// @lc code=end



/*
// @lcpr case=start
// [-1,1,2,3,1]\n2\n
// @lcpr case=end

// @lcpr case=start
// [-6,2,5,-2,-7,-1,3]\n-2\n
// @lcpr case=end

 */

