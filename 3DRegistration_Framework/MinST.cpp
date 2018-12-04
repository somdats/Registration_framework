/*!
	\file MinST.cpp
	
	Contains implementation of MST using Prim's algorithm
	Author: Divya Banda
	Email:	divyareddy@iitkgp.ac.in
*/

#pragma once
#include<malloc.h>
#include<stdio.h>
#include <vector>
#include <list>
#include"pch.h"
#include "Common.h"
#include"Graph.h"
#include "MinST.h"
#include "GraphVertex.h"

using namespace std;
// A utility function to create a new Min Heap Node
struct MinHeapNode* newMinHeapNode(int v, double key)
{
    /*struct MinHeapNode* minHeapNode =
           (struct MinHeapNode*) malloc(sizeof(struct MinHeapNode));*/
	struct MinHeapNode* minHeapNode = new MinHeapNode;
    minHeapNode->v = v;
    minHeapNode->key = key;
    return minHeapNode;
}
 
// A utilit function to create a Min Heap
struct MinHeap* createMinHeap(int capacity)
{
    /*struct MinHeap* minHeap =
         (struct MinHeap*) malloc(sizeof(struct MinHeap));*/
	struct MinHeap* minHeap = new MinHeap;
    /*minHeap->pos = (int *)malloc(capacity * sizeof(int));*/
	minHeap->pos = new int[capacity];
    minHeap->size = 0;
    minHeap->capacity = capacity;
    /*minHeap->Array =
         (struct MinHeapNode**) malloc(capacity * sizeof(struct MinHeapNode*));*/
	  minHeap->Array = new MinHeapNode* [capacity];

    return minHeap;
}
 
// A utility function to swap two nodes of min heap. Needed for min heapify
void swapMinHeapNode(struct MinHeapNode** a, struct MinHeapNode** b)
{
    struct MinHeapNode* t = *a;
    *a = *b;
    *b = t;
}
 
// A standard function to heapify at given idx
// This function also updates position of nodes when they are swapped.
// Position is needed for decreaseKey()
void minHeapify(struct MinHeap* minHeap, int idx)
{
    int smallest, left, right;
    smallest = idx;
    left = 2 * idx + 1;
    right = 2 * idx + 2;
 
    if (left < minHeap->size &&
        minHeap->Array[left]->key < minHeap->Array[smallest]->key )
      smallest = left;
 
    if (right < minHeap->size &&
        minHeap->Array[right]->key < minHeap->Array[smallest]->key )
      smallest = right;
 
    if (smallest != idx)
    {
        // The nodes to be swapped in min heap
        MinHeapNode *smallestNode = minHeap->Array[smallest];
        MinHeapNode *idxNode = minHeap->Array[idx];
 
        // Swap positions
        minHeap->pos[smallestNode->v] = idx;
        minHeap->pos[idxNode->v] = smallest;
 
        // Swap nodes
        swapMinHeapNode(&minHeap->Array[smallest], &minHeap->Array[idx]);
 
        minHeapify(minHeap, smallest);
    }
}
 
// A utility function to check if the given minHeap is ampty or not
int isEmpty(struct MinHeap* minHeap)
{
    return minHeap->size == 0;
}
 
// Standard function to extract minimum node from heap
struct MinHeapNode* extractMin(struct MinHeap* minHeap)
{
    if (isEmpty(minHeap))
        return NULL;
 
    // Store the root node
    struct MinHeapNode* root = minHeap->Array[0];
 
    // Replace root node with last node
    struct MinHeapNode* lastNode = minHeap->Array[minHeap->size - 1];
    minHeap->Array[0] = lastNode;
 
    // Update position of last node
    minHeap->pos[root->v] = minHeap->size-1;
    minHeap->pos[lastNode->v] = 0;
 
    // Reduce heap size and heapify root
    --minHeap->size;
    minHeapify(minHeap, 0);
 
    return root;
}
 
// Function to decreasy key value of a given vertex v. This function
// uses pos[] of min heap to get the current index of node in min heap
void decreaseKey(struct MinHeap* minHeap, int v, double key)
{
    // Get the index of v in  heap Array
    int i = minHeap->pos[v];
 
    // Get the node and update its key value
    minHeap->Array[i]->key = key;
 
    // Travel up while the complete tree is not hepified.
    // This is a O(Logn) loop
    while (i && minHeap->Array[i]->key < minHeap->Array[(i - 1) / 2]->key)
    {
        // Swap this node with its parent
        minHeap->pos[minHeap->Array[i]->v] = (i-1)/2;
        minHeap->pos[minHeap->Array[(i-1)/2]->v] = i;
        swapMinHeapNode(&minHeap->Array[i],  &minHeap->Array[(i - 1) / 2]);
 
        // move to parent index
        i = (i - 1) / 2;
    }
}
 
// A utility function to check if a given vertex
// 'v' is in min heap or not
bool isInMinHeap(struct MinHeap *minHeap, int v)
{
   if (minHeap->pos[v] < minHeap->size)
     return true;
   return false;
}
 
// A utility function used to print the constructed MST
void printArr( int arr[],int n)
{
    for (int i = 1; i < n; ++i)
        printf("%d - %d\n", i, arr[i]);
}

CGraphWithVertex Prim_MST(CGraphWithVertex grph)
{
	int V =  grph.GetNumVertices() ;
	int *parent;
	parent = new int[V];
	double *keyvalue;
	keyvalue =	new double [V];
	/* MinHeap *_Min;*/
	//MinHeapNode  _minnode;
	MinHeap *minheap;
	minheap = new MinHeap ;
	minheap = createMinHeap(V);
//	minheap->MinHeaparray.resize(V);
	//creating a graph representing MST
	
	// Initialize min heap with all vertices. Key value of
    // all vertices (except 0th vertex) is initially infinite
	for(int v = 1 ; v < V ; ++v)      //1
	{    
		// Rmst->SetVertexData( v ,v);    // -1
		parent[v] = -1;
		keyvalue[v] = INF;
		minheap->Array[v] = newMinHeapNode(v,keyvalue[v]);
		minheap->pos[v] =  v ;
	}
	keyvalue[0] = 0;
	minheap->Array[0] = newMinHeapNode(0,keyvalue[0]);
	minheap->pos[0] = 0;
	minheap->size =  V ;      // Initially size of min heap is equal to V
	// In the followin loop, min heap contains all nodes
    // not yet added to MST.
	//while(!minheap->isEmpty(*minheap))
	  while(!isEmpty(minheap))
	{
		//_minnode = minheap->extractMin(*minheap); 
		 // Extract the vertex with minimum key value
		//int u = _minnode.v ;                               // Store the extracted vertex number
		struct MinHeapNode* minHeapNode = extractMin(minheap);
		int u = minHeapNode->v;
		/*if(u == 426692 )
			int mm = 0;*/

		CGraphVertex<int> vertex ;
		std::list<CGraphEdge<int>>::iterator it;

		vertex = grph.GetVertex(u);
		//vertex = grph.m_cVertexList[u];
		it = vertex.m_cEdgeList.begin();
		while(it!= vertex.m_cEdgeList.end())
		{
			int t = it->m_nVertexIndex;

			if(isInMinHeap(minheap,t) && it->m_dEdgeWeight < keyvalue[t] )    //  minheap->MinHeaparray[t].key 	
			{  
				keyvalue[t] = it->m_dEdgeWeight ;
				parent[t] = u;                      //makes a  MST represented by adjacency list (u);
				decreaseKey(minheap ,t ,keyvalue[t]);  //   minheap->MinHeaparray[t].key
				
			}
			it++;
		}
	}
	 // printArr( parent, V );
	//printf("graph for dfs traversal\n");
	CGraphWithVertex *Rmst;
	Rmst = new CGraphWithVertex ;
	Rmst->Create(V,false);
	for(int k = 1 ; k < V ; k++)        //1
	{
		
		Rmst->SetVertexData(k,k);
		if(  parent[k]!= -1)
		{
	        Rmst->AddEdge(k , parent[k] ,keyvalue[k],0);
			//printf("%d - %d % f\n", k , parent[k], keyvalue[k]); 
		}
		

			
			// printf("%d ",parent[k]);
	}
		//printf("%d ",k);
	
	return (*Rmst) ;
	delete minheap;

}
//bool IsExistedge(int u, int v,CGraph &Cg)
//{
//	
//	CGraphVertex  Cnode;
//	Cnode = Cg.GetVertex(u);
//	std::list<CGraphEdge>::iterator it;
//
//	for(it =  Cnode.m_cEdgeList.begin(); it!= Cnode.m_cEdgeList.end(); ++it)
//	{
//
//	     int t = it->m_nVertexIndex;
//		 if( v == t)
//			 return true;
//		 return false;
//	}
//}
//bool IsEdge(int u, int v)
//{
//	CGraph Cg;
//	CGraphVertex  Cnode;
//	Cnode = Cg.GetVertex(u);
//	std::list<CGraphEdge>::iterator it;
//
//	for(it = it = vertex.m_cEdgeList.begin(); it!= vertex.m_cEdgeList.end(); ++it)
//	{
//
//	     int t = it->m_nVertexIndex;
//		 if( v == t)
//			 return true;
//		 return false;
//	}
//}

 