// @lcpr-before-debug-begin




// @lcpr-before-debug-end

/*
 * @lc app=leetcode.cn id=LCR 140 lang=cpp
 * @lcpr version=30121
 *
 * [LCR 140] 训练计划 II
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
/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 */
class Solution {
public:
    ListNode* trainingPlan(ListNode* head, int cnt) {
        ListNode* f_pt=head;
        ListNode* sec_pt=head;

        for(auto i = 0 ; i < cnt-1 ; i ++){
            if (sec_pt->next != nullptr){
                sec_pt = sec_pt->next;
            }else{
                return sec_pt;
            }
        }

        while(sec_pt->next != nullptr){
            f_pt=f_pt->next;
            sec_pt=sec_pt->next;
        }

        return f_pt;
    }
};
// @lc code=end


// @lcpr-div-debug-arg-start
// funName=trainingPlan
// paramTypes= ["number[]","number"]
// @lcpr-div-debug-arg-end




/*
// @lcpr case=start
// [2,4,7,8]\n1\n
// @lcpr case=end

 */

