/*!
	\file MinST.h
	
	Contains class declaration of MST
	Author: Divya Banda
	Email:	divyareddy@iitkgp.ac.in
*/

#pragma once
#include<malloc.h>
#include<stdio.h>
#include <vector>
#include <list>
#include "Common.h"
# pragma once
#include"Graph.h"
#include "GraphVertex.h"
// Structure to represent a min heap node
struct MinHeapNode
{
    int  v;
    double key;
};
 
// Structure to represent a min heap
struct MinHeap
{
    int size;      // Number of heap nodes present currently
    int capacity;  // Capacity of min heap
    int *pos;     // This is needed for decreaseKey()
    struct MinHeapNode **Array;
};
CGraphWithVertex Prim_MST(CGraphWithVertex grph);
 