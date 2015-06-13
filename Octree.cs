////'Hare: Accelerated Multi-Resolution Ray Tracing (GPL) by Arthur van der Harten
////'
////'Copyright (c) 2008, 2009, Arthur van der Harten			
////'This program is free software; you can redistribute it and/or modify
////'it under the terms of the GNU General Public License as published 
////'by the Free Software Foundation; either version 3 of the License, or
////'(at your option) any later version.
////'This program is distributed in the hope that it will be useful,
////'but WITHOUT ANY WARRANTY; without even the implied warranty of
////'MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
////'GNU General Public License for more details.
////'
////'You should have received a copy of the GNU General Public 
////'License along with this program; if not, write to the Free Software
////'Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;

//namespace Hare
//{
//    namespace Geometry
//    {
//        class Oct_Partition : Spatial_Partition
//        {
//            class Octree
//            {
//                Vector min;
//                Vector max;
//                Vector size;

//                //VoxelData data; // just here for reference: not really needed to know which data this tree covers
//                //DataPoint* voxeldata;

//                Stack<Node> nodes;
//                //Octree(VoxelData data, Vector min, Vector max, Vector size);
//                //Octree(DataPoint* voxeldata, Vector min, Vector max, Vector size);
                
//                public int storeNode(Node n)
//                {
//                    nodes.Push(n);
//                    return nodes.Count-1;
//                }

//                public Node getNode(int index)
//                {
//                    return nodes.ElementAt<Node>(index);
//                }

//                public Node getRootNode()
//                {
//                    return nodes.Peek();
//                }
//            }
            
//            // the struct we will use to communicate current traversal info
//            struct TraversalNodeInfo_ 
//            {
//                public Node node;
//                public Vector t0,t1,tm;
//                public int nextchild;
//            }

//            public class Node
//            {
                
//                public Node[] children = new Node[8];
//                public bool Terminal;
//                public bool hasdata;
//                public bool isNull;
//                public List<int> Poly_Ids = new List<int>;

//                public Node(AABB Bounds, Topology T, List<int> Polys)
//                {
//                    foreach (int p in Polys) if (Bounds.PolyBoxOverlap(T.Polygon_Vertices(p))) Poly_Ids.Add(p);

//                    if (Poly_Ids.Count > 0)
//                    {
//                        children[0] = new Node(new AABB(Bounds.Min_PT, Bounds.Center), T, Poly_Ids);
//                        children[1] = new Node(new AABB(new Point(Bounds.Min_PT.x, Bounds.Center.y, Bounds.Min.z), new Point(Bounds.Center.x, Bounds.Max_PT.y, Bounds.Center.z)), T, Poly_Ids);
//                        children[2] = new Node(new AABB(new Point(Bounds.Min_PT.x, Bounds.Min_PT.y, Bounds.Center.z), new Point(Bounds.Center.x, Bounds.Center.y, Bounds.Max_PT.z)), T, Poly_Ids);
//                        children[3] = new Node(new AABB(new Point(Bounds.Min_PT.x, Bounds.Center.y, Bounds.Center.z), new Point(Bounds.Center.x, Bounds.Max_PT.y, Bounds.Max_PT.z)), T, Poly_Ids);
//                        children[4] = new Node(new AABB(new Point(Bounds.Center.x, Bounds.Min_PT.y, Bounds.Min_PT.z), new Point(Bounds.Max_PT.x, Bounds.Center.y, Bounds.Center.z)), T, Poly_Ids);
//                        children[5] = new Node(new AABB(new Point(Bounds.Center.x, Bounds.Center.y, Bounds.Min.z), new Point(Bounds.Max_PT.x, Bounds.Max_PT.y, Bounds.Center.z)), T, Poly_Ids);
//                        children[6] = new Node(new AABB(new Point(Bounds.Center.x, Bounds.Min_PT.y, Bounds.Center.z), new Point(Bounds.Max_PT.x, Bounds.Center.y, Bounds.Max_PT.z)), T, Poly_Ids);
//                        children[7] = new Node(new AABB(Bounds.Center, Bounds.Max_PT), T, Poly_Ids);
//                    }
//                    Terminal = isTerminal();
//                    hasdata = hasData();
//                    isNull = isTerminal() && !hasData();;
//                }

//                public bool isTerminal()
//                {
//                    for(int i = 0; i<8; i++){if(children[i] != null){return false;}}
//                    return true;
//                }

//                public bool hasData()
//                {
//                    return Poly_Ids.Count > 0;
//                }
//            }

//            class TreeTraverser
//            {
//                public byte a;
//                public Octree octree;
//                public Ray ray;
//                public System.Collections.Stack stack;
//                //public List<TraversalNodeInfo> stack;
//                public int stepcount;

//                public TreeTraverser();
//                public TreeTraverser(Octree octree, Ray ray);
//                void step();
//                public bool isTerminated();
//                public Node getCurrentNode();
//                public Vector getCurrentPosition();

//                private TraversalNodeInfo_ buildNodeInfo(float tx0, float ty0, float tz0, float tx1, float ty1, float tz1, Node node);
//                private int newNode(float txm, int x, float tym, int y, float tzm, int z);
//                private int firstNode(float tx0, float ty0, float tz0, float txm, float tym, float tzm);
//                private void initTraversal();

//                int firstNode(float tx0, float ty0, float tz0, float txm, float tym, float tzm)
//                {
//                    byte answer = 0;       // initialize to 00000000
//                    // select the entry plane and set bits
//                    if(tx0 > ty0)
//                    {
//                        if(tx0 > tz0)
//                        { // PLANE YZ
//                            if(tym < tx0) answer|=2;        // set bit at position 1
//                            if(tzm < tx0) answer|=1;        // set bit at position 0
//                            return (int) answer;
//                        }
//                    } 
//                    else 
//                    {
//                        if(ty0 > tz0)
//                        { // PLANE XZ
//                            if(txm < ty0) answer|=4;        // set bit at position 2
//                            if(tzm < ty0) answer|=1;        // set bit at position 0
//                            return (int) answer;
//                        }
//                    }
//                    // PLANE XY
//                    if(txm < tz0) answer|=4;        // set bit at position 2
//                    if(tym < tz0) answer|=2;        // set bit at position 1
//                    return (int) answer;
//                }

//                // perform PUSH, POP or ADVANCE
//                public void step()
//                {
//                    stepcount++;
//                    // if the stack is empty, we're done
//                    if(stack.Count < 1) return;

//                    // define some aliases to make code readable
//                    Vector t0 = stack.Peek().t0;
//                    Vector t1 = stack.Peek().t1;
//                    Vector tm = stack.Peek().tm;

//                    //POP
//                    // if we're in a terminal node or if we visited all children of that node (next child = 8),
//                    // or if the t1 values have become negative, pop the current node from the stack
//                    if(stack.Peek().nextchild == 8 || stack.Peek().node->isTerminal())
//                    {
//                        stack.Pop();
//                        return;
//                    }

//                    // PUSH
//                    // we haven't looked at any child in this voxel yet: pick the first and push down
//                    if(stack.Peek().nextchild == -1)
//                    {
//                        // calculate midpoint and save it in stack
//                        tm = 0.5f*(t0+t1);
//                        // calculate first node
//                        stack.Peek().nextchild = firstNode(t0[0],t0[1],t0[2],tm[0],tm[1],tm[2]);
//                    }

//                    // ADVANCE
//                    // let's look at the next child in this voxel
//                    switch (stack.Peek().nextchild)
//                    {
//                        case 0: 
//                    stack.back().nextchild = newNode(tm[0],4,tm[1],2,tm[2],1);
//                    if(stack.back().node->children[a] != 0)
//                    {
//                        TraversalNodeInfo_ info = buildNodeInfo(t0[0],t0[1],t0[2],tm[0],tm[1],tm[2],octree->getNode(stack.back().node->children[a]));
//                        stack.push_back(info);
//                    }
//                    return;
//                case 1: 
//                    stack.back().nextchild = newNode(tm[0],5,tm[1],3,t1[2],8);
//                    if(stack.back().node->children[1^a] != 0)
//                    {
//                        TraversalNodeInfo_ info = buildNodeInfo(t0[0],t0[1],tm[2],tm[0],tm[1],t1[2],octree->getNode(stack.back().node->children[1^a]));
//                        stack.push_back(info);
//                    } 
//                    return;
//                case 2: 
//                    stack.back().nextchild = newNode(tm[0],6,t1[1],8,tm[2],3);
//                    if(stack.back().node->children[2^a] != 0)
//                    {
//                        TraversalNodeInfo_ info = buildNodeInfo(t0[0],tm[1],t0[2],tm[0],t1[1],tm[2],octree->getNode(stack.back().node->children[2^a]));
//                        stack.push_back(info);
//                    } return;
//                case 3: 
//                    stack.back().nextchild = newNode(tm[0],7,t1[1],8,t1[2],8);
//                    if(stack.back().node->children[3^a] != 0){
//                        TraversalNodeInfo_ info = buildNodeInfo(t0[0],tm[1],tm[2],tm[0],t1[1],t1[2],octree->getNode(stack.back().node->children[3^a]));
//                        stack.push_back(info);
//                    } 
//                    return;
//                case 4: 
//                    stack.back().nextchild = newNode(t1[0],8,tm[1],6,tm[2],5);
//                    if(stack.back().node->children[4^a] != 0)
//                    {
//                        TraversalNodeInfo_ info = buildNodeInfo(tm[0],t0[1],t0[2],t1[0],tm[1],tm[2],octree->getNode(stack.back().node->children[4^a]));
//                        stack.push_back(info);
//                    }
//                    return;
//                case 5: 
//                    stack.back().nextchild = newNode(t1[0],8,tm[1],7,t1[2],8);
//                    if(stack.back().node->children[5^a] != 0)
//                    {
//                        TraversalNodeInfo_ info = buildNodeInfo(tm[0],t0[1],tm[2],t1[0],tm[1],t1[2],octree->getNode(stack.back().node->children[5^a]));
//                        stack.push_back(info);
//                    } 
//                    return;
//                case 6: 
//                    stack.back().nextchild = newNode(t1[0],8,t1[1],8,tm[2],7);
//                    if(stack.back().node->children[6^a] != 0)
//                    {
//                        TraversalNodeInfo_ info = buildNodeInfo(tm[0],tm[1],t0[2],t1[0],t1[1],tm[2],octree->getNode(stack.back().node->children[6^a]));
//                        stack.push_back(info);
//                    } 
//                    return;
//                case 7: 
//                    stack.back().nextchild = 8;
//                    if(stack.back().node->children[7^a] != 0)
//                    {
//                        TraversalNodeInfo_ info = buildNodeInfo(tm[0],tm[1],tm[2],t1[0],t1[1],t1[2],octree->getNode(stack.back().node->children[7^a]));
//                        stack.push_back(info);
//                    }
//                    return;
//                }
//            }

//        void initTraversal()
//        {
//            stepcount = 0;
//            a = 0;

//            // fixes for rays with negative direction
//            if(ray.direction.x < 0)
//            {
//                ray.origin.x = octree. - ray.origin[0];
//                ray.direction[0] = - ray.direction[0];
//                a |= 4 ; //bitwise OR (latest bits are XYZ)
//            }
//            if(ray.direction[1] < 0)
//            {
//                ray.origin[1] = octree->size[1] - ray.origin[1];
//                ray.direction[1] = - ray.direction[1];
//                a |= 2 ;
//            }
//            if(ray.direction[2] > 0)
//            {
//                ray.origin[2] = octree->size[2] - ray.origin[2];
//                ray.direction[2] = - ray.direction[2];
//                a |= 1 ;
//            }

//            float tx0 = (octree->min[0] - ray.origin[0]) * (1 / ray.direction[0]);
//            float tx1 = (octree->max[0] - ray.origin[0]) * (1 / ray.direction[0]);
//            float ty0 = (octree->min[1] - ray.origin[1]) * (1 / ray.direction[1]);
//            float ty1 = (octree->max[1] - ray.origin[1]) * (1 / ray.direction[1]);
//            float tz0 = (octree->min[2] - ray.origin[2]) * (1 / ray.direction[2]);
//            float tz1 = (octree->max[2] - ray.origin[2]) * (1 / ray.direction[2]);

//            if( max(max(tx0,ty0),tz0) < min(min(tx1,ty1),tz1) )
//            {
//                // push root node on stack
//                stack.Push(buildNodeInfo(tx0,ty0,tz0,tx1,ty1,tz1,octree.getRootNode()));
//                return;
//            }
//        // push nothing on the stack
//        }


            

//    //inline TreeTraverser::TreeTraverser(Octree const* octree, Ray ray)
//    //    {
//    //        octree(octree), 
//    //        ray(ray){
//    //        initTraversal();
//    //}

//    public int newNode(float txm, int x, float tym, int y, float tzm, int z)
//    {
//        if(txm < tym)
//        {
//            if(txm < tzm){return x;}  // YZ plane
//        }
//        else 
//        {
//            if(tym < tzm){return y;} // XZ plane
//        }
//        return z; // XY plane;
//    }

//    public TraversalNodeInfo_ buildNodeInfo(float tx0, float ty0, float tz0, float tx1, float ty1, float tz1, Node node)
//    {
//        TraversalNodeInfo_ info;
//        info.node = node;
//        info.t0 = new Vector(tx0,ty0,tz0);
//        info.t1 = new Vector(tx1,ty1,tz1);
//        info.nextchild = -1;
//        return info;
//    }

//    public bool isTerminated() const
//    {
//        return (stack.empty());
//    }

//    public Vector getCurrentPosition()
//    {
//        float t = stack.Peek().t0.max();
//        return ray.getRayPoint(t);
//    }

//    public Node getCurrentNode() const
//    {
//        return stack.back().node;
//    }


//    //        private class Node
//    //        {

                
                
//    //            // Adapted From::
//    //            // http://www.forceflow.be/2012/05/10/ray-octree-traversal-parametric-algorithm-implementation/
//    //            //
//    //            // Ray-Octree intersection based on
//    //            // An Efficient Parametric Algorithm for Octree Traversal (2000)
//    //            // by J. Revelles , C. Ureña , M. Lastra
//    //            //
//    //            // Code by Jeroen Baert - www.forceflow.be - jeroen.baert@cs.kuleuven.be
//    //            // Licensed under a Creative Commons Attribute-NonCommercial Sharealike 3.0 Unported license
//    //            byte a; // because an unsigned char is 8 bits

//    //            public int first_node(double tx0, double ty0, double tz0, double txm, double tym, double tzm)
//    //            {
//    //                byte answer = 0;	// initialize to 00000000
//    //                // select the entry plane and set bits
//    //                if (tx0 > ty0)
//    //                {
//    //                    if (tx0 > tz0)
//    //                    { // PLANE YZ
//    //                        if (tym < tx0) answer |= 2;	// set bit at position 1
//    //                        if (tzm < tx0) answer |= 1;	// set bit at position 0 			return (int) answer; 		} 	} 	else { 		if(ty0 > tz0){ // PLANE XZ
//    //                        if (txm < ty0) answer |= 4;	// set bit at position 2
//    //                        if (tzm < ty0) answer |= 1;	// set bit at position 0
//    //                        return (int)answer;
//    //                    }
//    //                }
//    //                // PLANE XY
//    //                if (txm < tz0) answer |= 4;	// set bit at position 2
//    //                if (tym < tz0) answer |= 2;	// set bit at position 1
//    //                return (int)answer;
//    //            }

//    //            public int new_node(double txm, int x, double tym, int y, double tzm, int z)
//    //            {
//    //                if (txm < tym)
//    //                {
//    //                    if (txm < tzm) { return x; }  // YZ plane
//    //                }
//    //                else
//    //                {
//    //                    if (tym < tzm) { return y; } // XZ plane
//    //                }
//    //                return z; // XY plane;
//    //            }

//    //            public void proc_subtree(double tx0, double ty0, double tz0, double tx1, double ty1, double tz1, Node node)
//    //            {
//    //                double txm, tym, tzm;
//    //                int currNode;
 
//    //                if(tx1 < 0 || ty1 < 0 || tz1 < 0) return; 	
//    //                if(node.terminal)
//    //                {
//    //                    //cout << "Reached leaf node " << node->debug_ID << endl;
//    //                    return;
//    //                }
//    //                else
//    //                {
//    //                    //cout << "Reached node " << node->debug_ID << endl;
//    //                } 	
//    //                txm = 0.5*(tx0 + tx1); 	
//    //                tym = 0.5*(ty0 + ty1); 	
//    //                tzm = 0.5*(tz0 + tz1); 	
//    //                currNode = first_node(tx0,ty0,tz0,txm,tym,tzm); 	
//    //                do
//    //                { 		
//    //                    switch (currNode) 		
//    //                    { 		
//    //                        case 0: 
//    //                            proc_subtree(tx0,ty0,tz0,txm,tym,tzm,node.children[a]);
//    //                            currNode = new_node(txm,4,tym,2,tzm,1);
//    //                            break;
//    //                    case 1: 
//    //                        proc_subtree(tx0,ty0,tzm,txm,tym,tz1,node.children[1^a]);
//    //                        currNode = new_node(txm,5,tym,3,tz1,8);
//    //                        break;        
//    //                case 2:                 
//    //                    proc_subtree(tx0,tym,tz0,txm,ty1,tzm,node.children[2^a]);
//    //                    currNode = new_node(txm,6,ty1,8,tzm,3);
//    //                    break;        
//    //                case 3: 
//    //                    proc_subtree(tx0,tym,tzm,txm,ty1,tz1,node.children[3^a]);
//    //                    currNode = new_node(txm,7,ty1,8,tz1,8);
//    //                    break;
//    //                case 4: 
//    //                    proc_subtree(txm,ty0,tz0,tx1,tym,tzm,node.children[4^a]);
//    //                    currNode = new_node(tx1,8,tym,6,tzm,5);
//    //                    break;
//    //                case 5: 
//    //                    proc_subtree(txm,ty0,tzm,tx1,tym,tz1,node.children[5^a]);
//    //                    currNode = new_node(tx1,8,tym,7,tz1,8);
//    //                    break;
//    //                case 6: 
//    //                    proc_subtree(txm,tym,tz0,tx1,ty1,tzm,node.children[6^a]);
//    //                    currNode = new_node(tx1,8,ty1,8,tzm,7);
//    //                    break;
//    //                case 7: 
//    //                    proc_subtree(txm,tym,tzm,tx1,ty1,tz1,node.children[7^a]);
//    //                    currNode = 8;
//    //                    break;
//    //                    }
//    //                } while (
//    //                    currNodesize[0] - ray.origin[0];
//    //                    ray.direction[0] = - ray.direction[0];
//    //                    a |= 4 ; //bitwise OR (latest bits are XYZ)
//    //if(ray.direction[1] < 0)
//    //{ 		
//    //    ray.origin[1] = octree->size[1] - ray.origin[1];
//    //    ray.direction[1] = - ray.direction[1];
//    //    a |= 2 ;
//    //}
//    //if(ray.direction[2] < 0)
//    //{ 		
//    //    ray.origin[2] = octree->size[2] - ray.origin[2];
//    //    ray.direction[2] = - ray.direction[2];
//    //    a |= 1 ;
//    //}
 
//    //double divx = 1 / ray.direction.x; // IEEE stability fix
//    //double divy = 1 / ray.direction.y;
//    //double divz = 1 / ray.direction.z;
 
//    //double tx0 = (octree.min.x - ray.origin.x) * divx;
//    //double tx1 = (octree.max.x - ray.origin.x) * divx;
//    //double ty0 = (octree.min.y - ray.origin.y) * divy;
//    //double ty1 = (octree.max.y - ray.origin.y) * divy;
//    //double tz0 = (octree.min.z - ray.origin.z) * divz;
//    //double tz1 = (octree.max.z - ray.origin.z) * divz;
 
//    //if( max(max(tx0,ty0),tz0) < min(min(tx1,ty1),tz1) )
//    //{ 		
//    //    proc_subtree(tx0,ty0,tz0,tx1,ty1,tz1,octree.root);
//    //});
//            //    }
//            //}
//        //}
//    }
//}}