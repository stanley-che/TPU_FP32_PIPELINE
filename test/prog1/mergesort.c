#include <stdio.h>
#include <stdlib.h>

// 分治-治
void mergeSort_conquer(int* array, int left, int mid, int right, int* temp) {
    // [left, mid]和[mid+1, right]两个有序数组
    int i = left;
    int j = mid + 1;
    int index = 0;
    while (i <= mid && j <= right) {
        if (array[i] < array[j]) {
            temp[index++] = array[i++];
        } else {
            temp[index++] = array[j++];
        }
    }
    // 剩余元素直接放入temp
    while (i <= mid) {
        temp[index++] = array[i++];
    }
    while (j <= right) {
        temp[index++] = array[j++];
    }
    // 放回原数组
    index = 0;
    while (left <= right) {
        array[left++] = temp[index++];
    }
}

// 分治-分
void mergeSort_divide(int* array, int left, int right, int* temp) {
    if (left < right) {
        int mid = left + (right - left) / 2;
        // 左边归并排序
        mergeSort_divide(array, left, mid, temp);
        // 右边归并排序
        mergeSort_divide(array, mid + 1, right, temp);
        // 合并两个有序序列
        mergeSort_conquer(array, left, mid, right, temp);
    }
}

void mergeSort(int* array, int size) {
    int* temp = (int*)malloc(sizeof(int) * size);
    mergeSort_divide(array, 0, size - 1, temp);
}

int main(){
  int array[8]={5,4,6,3,7,2,8,1};
  mergeSort(array,8);
  for(int i=0;i<8;i++){
    printf("%d",array[i]);
  }
  return 0;
}
