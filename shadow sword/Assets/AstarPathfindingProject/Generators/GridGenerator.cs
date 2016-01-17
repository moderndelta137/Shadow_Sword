//#define ASTAR_NoTagPenalty		//@SHOWINEDITOR Enables or disables tag penalties. Can give small performance boost
//#define ASTARDEBUG
//#define ASTAR_NO_JSON
#define ASTAR_GRID_CUSTOM_CONNECTIONS	

//#if UNITY_WINRT && !UNITY_EDITOR
//using Math = MarkerMetro.Unity.WinLegacy.Math;
//#else
using Math = System.Math;
//#endif
	
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding.Nodes;
using Pathfinding.Serialization.JsonFx;
using Pathfinding.Serialization;

namespace Pathfinding {
	[JsonOptIn]
	/** Generates a grid of nodes.
The GridGraph does exactly what the name implies, generates nodes in a grid pattern.\n
Grid graphs suit well to when you already have a grid based world.
Features:
 - You can update the graph during runtime (good for e.g Tower Defence or RTS games)
 - Throw any scene at it, with minimal configurations you can get a good graph from it.
 - Supports raycast and the funnel algorithm
 - Predictable pattern
 - Can apply penalty and walkability values from a supplied image
 - Perfect for terrain worlds since it can make areas unwalkable depending on the slope

\shadowimage{gridgraph_graph.png}
\shadowimage{gridgraph_inspector.png}

The <b>The Snap Size</b> button snaps the internal size of the graph to exactly contain the current number of nodes, i.e not contain 100.3 nodes but exactly 100 nodes.\n
This will make the "center" coordinate more accurate.\n

<b>The Graph During Runtime</b>
Any graph which implements the IUpdatableGraph interface can be updated during runtime.\n
For grid graphs this is a great feature since you can update only a small part of the grid without causing any lag like a complete rescan would.\n
If you for example just have instantiated a sphere obstacle in the scene and you want to update the grid where that sphere was instantiated, you can do this:\n
\code AstarPath.active.UpdateGraphs (ob.collider.bounds); \endcode
Where \a ob is the obstacle you just instantiated (a GameObject).\n
As you can see, the UpdateGraphs function takes a Bounds parameter and it will send an update call to all updateable graphs.\n
A grid graph will update that area and a small margin around it equal to \link Pathfinding.GraphCollision.diameter collision testing diameter/2 \endlink
\see graph-updates for more info about updating graphs during runtime


<b>Configure using script</b>
\code
// This holds all graph data
AstarData data = AstarPath.active.astarData;

// This creates a Grid Graph
GridGraph gg = data.AddGraph(typeof(GridGraph)) as GridGraph;

// Setup a grid graph with some values
gg.width = 50;
gg.depth = 50;
gg.nodeSize = 1;
gg.center = new Vector3 (10,0,0);

// Updates internal size from the above values
gg.UpdateSizeFromWidthDepth();

// Scans all graphs, do not call gg.Scan(), that is an internal method
AstarPath.active.Scan();
\endcode

\ingroup graphs
\nosubgrouping
	 */
	public class GridGraph : NavGraph, IUpdatableGraph
	,IRaycastableGraph
	{
		
		/** This function will be called when this graph is destroyed */
		public override void OnDestroy () {
			
			base.OnDestroy ();
			
			//Clean up a reference in a static variable which otherwise should point to this graph forever and stop the GC from collecting it
			RemoveGridGraphFromStatic ();
		}
		
		public void RemoveGridGraphFromStatic () {
			GridNode.SetGridGraph (AstarPath.active.astarData.GetGraphIndex(this),null);
		}
		
		/** This is placed here so generators inheriting from this one can override it and set it to false.
		If it is true, it means that the nodes array's length will always be equal to width*depth
		It is used mainly in the editor to do auto-scanning calls, setting it to false for a non-uniform grid will reduce the number of scans */
		public virtual bool uniformWidthDepthGrid {
			get {
				return true;
			}
		}
		
		public override void GetNodes (GraphNodeDelegateCancelable del) {
			if (nodes == null) return;
			for (int i=0;i<nodes.Length && del (nodes[i]);i++) {}
		}
		
		/** \name Inspector - Settings
		 * \{ */
		
		public int width; /**< Width of the grid in nodes. \see UpdateSizeFromWidthDepth */
		public int depth; /**< Depth (height) of the grid in nodes. \see UpdateSizeFromWidthDepth */
		
		[JsonMember]
		public float aspectRatio = 1F; /**< Scaling of the graph along the X axis. This should be used if you want different scales on the X and Y axis of the grid */

		/** Angle to use for the isometric projection.
		 * If you are making a 2D isometric game, you may want to use this parameter to adjust the layout of the graph to match your game.
		 * This will essentially scale the graph along one of its diagonals to produce something like this:
		 * 
		 * A perspective view of an isometric graph.
		 * \shadowimage{isometric/isometric_perspective.png}
		 * 
		 * A top down view of an isometric graph. Note that the graph is entirely 2D, there is no perspective in this image.
		 * \shadowimage{isometric/isometric_top.png}
		 * 
		 * Usually the angle that you want to use is either 30 degrees (alternatively 90-30 = 60 degrees) or atan(1/sqrt(2)) which is approximately 35.264 degrees (alternatively 90 - 35.264 = 54.736 degrees).
		 * You might also want to rotate the graph plus or minus 45 degrees around the Y axis to get the oritientation required for your game.
		 * 
		 * You can read more about it on the wikipedia page linked below.
		 * 
		 * \see http://en.wikipedia.org/wiki/Isometric_projection
		 * \see rotation
		 */
		[JsonMember]
		public float isometricAngle = 0;

		[JsonMember]
		public Vector3 rotation; /**< Rotation of the grid in degrees */
		
		public Bounds bounds;
		
		[JsonMember]
		public Vector3 center; /**< Center point of the grid */
		
		[JsonMember]
		public Vector2 unclampedSize; 	/**< Size of the grid. Might be negative or smaller than #nodeSize */
		
		[JsonMember]
		public float nodeSize = 1; /**< Size of one node in world units. \see UpdateSizeFromWidthDepth */
		
		/* Collision and stuff */
		
		/** Settings on how to check for walkability and height */
		[JsonMember]
		public GraphCollision collision;
		
		/** The max position difference between two nodes to enable a connection. Set to 0 to ignore the value.*/
		[JsonMember]
		public float maxClimb = 0.4F;
		
		/** The axis to use for #maxClimb. X = 0, Y = 1, Z = 2. */
		[JsonMember]
		public int maxClimbAxis = 1;
		
		/** The max slope in degrees for a node to be walkable. */
		[JsonMember]
		public float maxSlope = 90;
		
		/** Use heigh raycasting normal for max slope calculation.
		 * True if #maxSlope is less than 90 degrees.
		 */
		public bool useRaycastNormal { get { return System.Math.Abs (90-maxSlope) > float.Epsilon; }}
		
		/** Erosion of the graph.
		 * The graph can be eroded after calculation.
		 * This means a margin is put around unwalkable nodes or other unwalkable connections.
		 * It is really good if your graph contains ledges where the nodes without erosion are walkable too close to the edge.
		 * 
		 * Below is an image showing a graph with erode iterations 0, 1 and 2
		 * \shadowimage{erosion.png}
		 * 
		 * \note A high number of erode iterations can seriously slow down graph updates during runtime (GraphUpdateObject)
		 * and should be kept as low as possible.
		 * \see erosionUseTags
		 */
		[JsonMember]
		public int erodeIterations = 0;
		
		/** Use tags instead of walkability for erosion.
		 * Tags will be used for erosion instead of marking nodes as unwalkable. The nodes will be marked with tags in an increasing order starting with the tag #erosionFirstTag.
		 * Debug with the Tags mode to see the effect. With this enabled you can in effect set how close different AIs are allowed to get to walls using the Valid Tags field on the Seeker component. 
		 * \shadowimage{erosionTags.png}
		 * \shadowimage{erosionTags2.png}
		 * \see erosionFirstTag
		 */
		[JsonMember]
		public bool erosionUseTags = false;
		
		/** Tag to start from when using tags for erosion.
		 * \see #erosionUseTags
		 * \see #erodeIterations
		 */
		[JsonMember]
		public int erosionFirstTag = 1;
		
		/** 
		 * Auto link the graph's edge nodes together with other GridGraphs in the scene on Scan.
		 * \warning This feature is experimental and it is currently disabled.
		 *
		 * \see #autoLinkDistLimit */
		[JsonMember]
		public bool autoLinkGrids = false;
		
		/**
		 * Distance limit for grid graphs to be auto linked.
		 * \warning This feature is experimental and it is currently disabled.
		 *
		 * \see #autoLinkGrids */
		[JsonMember]
		public float autoLinkDistLimit = 10F;
		
		/** Number of neighbours for each node. Either four directional or eight directional */
		[JsonMember]
		public NumNeighbours neighbours = NumNeighbours.Eight;
		
		/** If disabled, will not cut corners on obstacles. If \link #neighbours connections \endlink is Eight, obstacle corners might be cut by a connection, setting this to false disables that. \image html images/cutCorners.png */
		[JsonMember]
		public bool cutCorners = true;
		
		/** Offset for the position when calculating penalty.
		  * \see penaltyPosition */
		[JsonMember]
		public float penaltyPositionOffset = 0;

		/** Use position (y-coordinate) to calculate penalty */
		[JsonMember]

		public bool penaltyPosition = false;
		/** Scale factor for penalty when calculating from position.
		 * \see penaltyPosition
		 */
		[JsonMember]
		public float penaltyPositionFactor = 1F;
		
		[JsonMember]
		public bool penaltyAngle = false;

		[JsonMember]
		/** How much penalty is applied depending on the slope of the terrain.
		  * At a 90 degree slope (not that exactly 90 degree slopes can occur, but almost 90 degree), this penalty is applied.
		  * At a 45 degree slope, half of this is applied and so on.
		  * Note that you may require very large values, a value of 1000 is equivalent to the cost of moving 1 world unit.
		  */
		public float penaltyAngleFactor = 100F;

		[JsonMember]
		/** How much extra to penalize very steep angles.
		 */
		public float penaltyAnglePower = 1;

		[JsonMember]
		public bool useJumpPointSearch = false;

		/** Holds settings for using a texture as source for a grid graph.
		 * Texure data can be used for fine grained control over how the graph will look.
		 * It can be used for positioning, penalty and walkability control.\n
		 * Below is a screenshot of a grid graph with a penalty map applied.
		 * It has the effect of the AI taking the longer path along the green (low penalty) areas.\n
		 * \shadowimage{penaltymap.png}
		 * Color data is got as 0...255 values.
		 * \astarpro
		 * \warning Can only be used with Unity 3.4 and up */
		[JsonMember]
		public TextureData textureData = new TextureData ();
		
		/** \} */
		
		public Vector2 size;			/**< Size of the grid. Will always be positive and larger than #nodeSize. \see #GenerateMatrix */
		
		/* End collision and stuff */
		
		/** Index offset to get neighbour nodes. Added to a node's index to get a neighbour node index */
		[System.NonSerialized]
		public readonly int[] neighbourOffsets = new int[8];
		
		/** Costs to neighbour nodes */
		[System.NonSerialized]
		public readonly uint[] neighbourCosts = new uint[8];
		
		/** Offsets in the X direction for neighbour nodes. Only 1, 0 or -1 */
		[System.NonSerialized]
		public readonly int[] neighbourXOffsets = new int[8];
		
		/** Offsets in the Z direction for neighbour nodes. Only 1, 0 or -1 */
		[System.NonSerialized]
		public readonly int[] neighbourZOffsets = new int[8];
		
		/* Same as #nodes, but already in the correct type (GridNode instead of Node) */
		//public GridNode[] graphNodes;
		
		/* If a walkable node wasn't found, then it will search (max) in a square with the side of 2*getNearestForceLimit+1 for a close walkable node */
		//public int getNearestForceLimit = 40;
		/** In GetNearestForce, determines how far to search after a valid node has been found */
		public const int getNearestForceOverlap = 2;
		
		public Matrix4x4 boundsMatrix;
		public Matrix4x4 boundsMatrix2;
		
		public int scans = 0;
		
		/** All nodes in this graph */
		public GridNode[] nodes;
		
		
		/** Used for using a texture as a source for a grid graph.
		 * \astarpro
		 * \warning Can only be used with Unity 3.4 and up */
		public class TextureData {
			public bool enabled = false;
			
			public Texture2D source = null;
			public float[] factors = new float[3];
			public ChannelUse[] channels = new ChannelUse[3];
			
#if !UNITY_3_3
			Color32[] data = null;
#endif
			
			/** Reads texture data */
			public void Initialize () {
				if (enabled && source != null) {
#if !UNITY_3_3
					for (int i=0;i<channels.Length;i++) {
						if (channels[i] != ChannelUse.None) {
							try {
								data = source.GetPixels32 ();
							} catch (UnityException e) {
								Debug.LogWarning (e.ToString ());
								data = null;
							}
							break;
						}
					}
#else
					Debug.LogError ("Cannot use textures in Unity 3.3, please upgrade to a newer version of Unity");
#endif
				}
			}
			
			/** Applies the texture to the node */
			public void Apply (GridNode node, int x, int z) {
#if !UNITY_3_3
				if (enabled && data != null && x < source.width && z < source.height) {
					Color32 col = data[z*source.width+x];
					
					if (channels[0] != ChannelUse.None) {
						ApplyChannel (node,x,z,col.r,channels[0],factors[0]);
					}
					
					if (channels[1] != ChannelUse.None) {
						ApplyChannel (node,x,z,col.g,channels[1],factors[1]);
					}
					
					if (channels[2] != ChannelUse.None) {
						ApplyChannel (node,x,z,col.b,channels[2],factors[2]);
					}
				}
#endif
			}
			
			/** Applies a value to the node using the specified ChannelUse */
			void ApplyChannel (GridNode node, int x, int z, int value, ChannelUse channelUse, float factor) {
				switch (channelUse) {
				case ChannelUse.Penalty:
					node.Penalty += (uint)Mathf.RoundToInt (value*factor);
					break;
				case ChannelUse.Position:
					node.position = GridNode.GetGridGraph(node.GraphIndex).GetNodePosition (node.NodeInGridIndex, Mathf.RoundToInt (value*factor*Int3.Precision));
					break;
				case ChannelUse.WalkablePenalty:
					if (value == 0) {
						node.Walkable = false;
					} else {
						node.Penalty += (uint)Mathf.RoundToInt ((value-1)*factor);
					}
					break;
				}
			}
			
			public enum ChannelUse {
				None,
				Penalty,
				Position,
				WalkablePenalty,
			}
		}
		
		public GridGraph () {
			unclampedSize = new Vector2 (10,10);
			nodeSize = 1F;
			collision = new GraphCollision ();
		}
		
		public Int3 GetNodePosition (int index, int yOffset) {
			int z = index/Width;
			int x = index - z*Width;
			return (Int3)matrix.MultiplyPoint3x4(new Vector3(x+0.5f,yOffset*Int3.PrecisionFactor,z+0.5f));//return Int3.zero;}
		}
		
		public int Width {
			get {
				return width; 
			}
			set {
				width = value;
			}
		}
		public int Depth {
			get {
				return depth; 
			}
			set {
				depth = value;
			}
		}
		
		public uint GetConnectionCost (int dir) {
			return neighbourCosts[dir];
		}
		
		public GridNode GetNodeConnection (GridNode node, int dir) {
			if (!node.GetConnectionInternal(dir)) return null;
			else if (!node.EdgeNode) {
				return nodes[node.NodeInGridIndex + neighbourOffsets[dir]];
			} else {
				int index = node.NodeInGridIndex;
				//int z = Math.DivRem (index,Width, out x);
				int z = index/Width;
				int x = index - z*Width;
				
				return GetNodeConnection (index, x, z, dir);
			}
		}
		
		public bool HasNodeConnection (GridNode node, int dir) {
			if (!node.GetConnectionInternal(dir)) return false;
			else if (!node.EdgeNode) {
				return true;
			} else {
				int index = node.NodeInGridIndex;
				int z = index/Width;
				int x = index - z*Width;
				
				return HasNodeConnection (index, x, z, dir);
			}
		}
		
		public void SetNodeConnection (GridNode node, int dir, bool value) {
			int index = node.NodeInGridIndex;
			int z = index/Width;
			int x = index - z*Width;
			
			SetNodeConnection (index, x, z, dir, value);
		}
		
		/** Get the connecting node from the node at (x,z) in the specified direction.
		 * \returns A GridNode if the node has a connection to that node. Null if no connection in that direction exists
		 * 
		 * \see GridNode
		 */
		private GridNode GetNodeConnection (int index, int x, int z, int dir) {
			if (!nodes[index].GetConnectionInternal (dir)) return null;
			
			/** \todo Mark edge nodes and only do bounds checking for them */
			int nx = x + neighbourXOffsets[dir];
			if (nx < 0 || nx >= Width) return null; /** \todo Modify to get adjacent grid graph here */
			int nz = z + neighbourZOffsets[dir];
			if (nz < 0 || nz >= Depth) return null;
			int nindex = index + neighbourOffsets[dir];
			
			//if (dir < 8) return nodes[index].GetConnectionInternal (dir) ? nodes[nindex] : null;
			return nodes[nindex];
		}
		
		/** Set if connection in the specified direction should be enabled.
		 * Note that bounds checking will still be done when getting the connection value again,
		 * so it is not necessarily true that HasNodeConnection will return true just because you used
		 * SetNodeConnection on a node to set a connection to true.
		 * 
		 * \param index Index of the node
		 * \param x X coordinate of the node
		 * \param z Z coordinate of the node
		 * \param value Enable or disable the connection
		 * 
		 * \note This is identical to Pathfinding.Node.SetConnectionInternal
		 * 
		 * \deprecated
		 */
		public void SetNodeConnection (int index, int x, int z, int dir, bool value) {
			nodes[index].SetConnectionInternal (dir, value);
		}
		
		public bool HasNodeConnection (int index, int x, int z, int dir) {
			if (!nodes[index].GetConnectionInternal (dir)) return false;
			
			/** \todo Mark edge nodes and only do bounds checking for them */
			int nx = x + neighbourXOffsets[dir];
			if (nx < 0 || nx >= Width) return false; /** \todo Modify to get adjacent grid graph here */
			int nz = z + neighbourZOffsets[dir];
			if (nz < 0 || nz >= Depth) return false;
			
			return true;
		}
		
		/** Updates #size from #width, #depth and #nodeSize values. Also \link GenerateMatrix generates a new matrix \endlink.
		 * \note This does not rescan the graph, that must be done with Scan */
		public void UpdateSizeFromWidthDepth () {
			unclampedSize = new Vector2 (width,depth)*nodeSize;
			GenerateMatrix ();
		}
		
		/** Generates the matrix used for translating nodes from grid coordinates to world coordintes. */
		public void GenerateMatrix () {
			
			size = unclampedSize;
			
			size.x *= Mathf.Sign (size.x);
			//size.y *= Mathf.Sign (size.y);
			size.y *= Mathf.Sign (size.y);
			
			//Clamp the nodeSize at 0.1
			nodeSize = Mathf.Clamp (nodeSize,size.x/1024F,Mathf.Infinity);//nodeSize < 0.1F ? 0.1F : nodeSize;
			nodeSize = Mathf.Clamp (nodeSize,size.y/1024F,Mathf.Infinity);
			
			size.x = size.x < nodeSize ? nodeSize : size.x;
			//size.y = size.y < 0.1F ? 0.1F : size.y;
			size.y = size.y < nodeSize ? nodeSize : size.y;
			
			var isometricMatrix = Matrix4x4.TRS (Vector3.zero, Quaternion.Euler(0,45,0), Vector3.one);
			isometricMatrix = Matrix4x4.Scale ( new Vector3 ( Mathf.Cos(Mathf.Deg2Rad*isometricAngle), 1, 1 ) ) * isometricMatrix;
			isometricMatrix = Matrix4x4.TRS (Vector3.zero, Quaternion.Euler(0,-45,0), Vector3.one) * isometricMatrix;
			boundsMatrix = Matrix4x4.TRS (center,Quaternion.Euler (rotation),new Vector3 (aspectRatio,1,1)) * isometricMatrix;
			
			width = Mathf.FloorToInt (size.x / nodeSize);
			depth = Mathf.FloorToInt (size.y / nodeSize);
			
			if (Mathf.Approximately (size.x / nodeSize,Mathf.CeilToInt (size.x / nodeSize))) {
				width = Mathf.CeilToInt (size.x / nodeSize);
			}
			
			if (Mathf.Approximately (size.y / nodeSize,Mathf.CeilToInt (size.y / nodeSize))) {
				depth = Mathf.CeilToInt (size.y / nodeSize);
			}
			
			//height = size.y;


			var m = Matrix4x4.TRS (boundsMatrix.MultiplyPoint3x4 (-new Vector3 (size.x,0,size.y)*0.5F),Quaternion.Euler(rotation), new Vector3 (nodeSize*aspectRatio,1,nodeSize)) * isometricMatrix;

			SetMatrix (m);
		}
		
		/** \todo Set clamped position for Grid Graph */
		public override NNInfo GetNearest (Vector3 position, NNConstraint constraint, GraphNode hint) {
			
			if (nodes == null || depth*width != nodes.Length) {
				return new NNInfo ();
			}
			
			position = inverseMatrix.MultiplyPoint3x4 (position);
			
			float xf = position.x-0.5F;
			float zf = position.z-0.5f;
			int x = Mathf.Clamp (Mathf.RoundToInt (xf)  , 0, width-1);
			int z = Mathf.Clamp (Mathf.RoundToInt (zf)  , 0, depth-1);
			
			NNInfo nn = new NNInfo(nodes[z*width+x]);
			
			float y = inverseMatrix.MultiplyPoint3x4((Vector3)nodes[z*width+x].position).y;
			nn.clampedPosition = matrix.MultiplyPoint3x4 (new Vector3(Mathf.Clamp(xf,x-0.5f,x+0.5f)+0.5f,y,Mathf.Clamp(zf,z-0.5f,z+0.5f)+0.5f));
			
			return nn;
		}
		
		/** \todo Set clamped position for Grid Graph */
		public override NNInfo GetNearestForce (Vector3 position, NNConstraint constraint) {
			
			if (nodes == null || depth*width != nodes.Length) {
				return new NNInfo ();
			}
			
			Vector3 globalPosition = position;
			
			position = inverseMatrix.MultiplyPoint3x4 (position);
			
			float xf = position.x-0.5F;
			float zf = position.z-0.5f;
			int x = Mathf.Clamp (Mathf.RoundToInt (xf)  , 0, width-1);
			int z = Mathf.Clamp (Mathf.RoundToInt (zf)  , 0, depth-1);
			
			GridNode node = nodes[x+z*width];
			
			GridNode minNode = null;
			float minDist = float.PositiveInfinity;
			int overlap = getNearestForceOverlap;
			
			Vector3 clampedPosition = Vector3.zero;
			NNInfo nn = new NNInfo(null);
			
			if (constraint.Suitable (node)) {
				minNode = node;
				minDist = ((Vector3)minNode.position-globalPosition).sqrMagnitude;
				float y = inverseMatrix.MultiplyPoint3x4((Vector3)node.position).y;
				clampedPosition = matrix.MultiplyPoint3x4 (new Vector3(Mathf.Clamp(xf,x-0.5f,x+0.5f)+0.5f, y, Mathf.Clamp(zf,z-0.5f,z+0.5f)+0.5f));
			}
			
			if (minNode != null) {
				nn.node = minNode;
				nn.clampedPosition = clampedPosition;
				
				if (overlap == 0) return nn;
				else overlap--;
			}
			
			float maxDist = constraint.constrainDistance ? AstarPath.active.maxNearestNodeDistance : float.PositiveInfinity;
			float maxDistSqr = maxDist*maxDist;
			

			for (int w = 1;;w++) {
				
				//Check if the nodes are within distance limit
				if (nodeSize*w > maxDist) {
					nn.node = minNode;
					nn.clampedPosition = clampedPosition;
					return nn;
				}
				
				bool anyInside = false;
				
				int nx = x;
				int nz = z+w;
				
				int nz2 = nz*width;
				
				for (nx = x-w;nx <= x+w;nx++) {
					if (nx < 0 || nz < 0 || nx >= width || nz >= depth) continue;
					anyInside = true;
					if (constraint.Suitable (nodes[nx+nz2])) {
						float dist = ((Vector3)nodes[nx+nz2].position-globalPosition).sqrMagnitude;
						//Debug.DrawRay (nodes[nx+nz2].position,Vector3.up*dist,Color.cyan);counter++;
						if (dist < minDist && dist < maxDistSqr) {
							minDist = dist;
							minNode = nodes[nx+nz2];
							clampedPosition = matrix.MultiplyPoint3x4 (new Vector3 (Mathf.Clamp(xf,nx-0.5f,nx+0.5f)+0.5f, inverseMatrix.MultiplyPoint3x4((Vector3)minNode.position).y, Mathf.Clamp(zf,nz-0.5f,nz+0.5f)+0.5f));
						}
					}
				}
				
				nz = z-w;
				nz2 = nz*width;
				
				for (nx = x-w;nx <= x+w;nx++) {
					if (nx < 0 || nz < 0 || nx >= width || nz >= depth) continue;
					anyInside = true;
					if (constraint.Suitable (nodes[nx+nz2])) {
						float dist = ((Vector3)nodes[nx+nz2].position-globalPosition).sqrMagnitude;
						//Debug.DrawRay (nodes[nx+nz2].position,Vector3.up*dist,Color.cyan);counter++;
						if (dist < minDist && dist < maxDistSqr) {
							minDist = dist;
							minNode = nodes[nx+nz2];
							clampedPosition = matrix.MultiplyPoint3x4 (new Vector3 (Mathf.Clamp(xf,nx-0.5f,nx+0.5f)+0.5f, inverseMatrix.MultiplyPoint3x4((Vector3)minNode.position).y, Mathf.Clamp(zf,nz-0.5f,nz+0.5f)+0.5f));
						}
					}
				}
				
				nx = x-w;
				nz = z-w+1;
				
				for (nz = z-w+1;nz <= z+w-1; nz++) {
					if (nx < 0 || nz < 0 || nx >= width || nz >= depth) continue;
					anyInside = true;
					if (constraint.Suitable (nodes[nx+nz*width])) {
						float dist = ((Vector3)nodes[nx+nz*width].position-globalPosition).sqrMagnitude;
						//Debug.DrawRay (nodes[nx+nz*width].position,Vector3.up*dist,Color.cyan);counter++;
						if (dist < minDist && dist < maxDistSqr) {
							minDist = dist;
							minNode = nodes[nx+nz*width];
							clampedPosition = matrix.MultiplyPoint3x4 (new Vector3 (Mathf.Clamp(xf,nx-0.5f,nx+0.5f)+0.5f, inverseMatrix.MultiplyPoint3x4((Vector3)minNode.position).y, Mathf.Clamp(zf,nz-0.5f,nz+0.5f)+0.5f));
						}
					}
				}
				
				nx = x+w;
				
				for (nz = z-w+1;nz <= z+w-1; nz++) {
					if (nx < 0 || nz < 0 || nx >= width || nz >= depth) continue;
					anyInside = true;
					if (constraint.Suitable (nodes[nx+nz*width])) {
						float dist = ((Vector3)nodes[nx+nz*width].position-globalPosition).sqrMagnitude;
						//Debug.DrawRay (nodes[nx+nz*width].position,Vector3.up*dist,Color.cyan);counter++;
						if (dist < minDist && dist < maxDistSqr) {
							minDist = dist;
							minNode = nodes[nx+nz*width];
							clampedPosition = matrix.MultiplyPoint3x4 (new Vector3 (Mathf.Clamp(xf,nx-0.5f,nx+0.5f)+0.5f, inverseMatrix.MultiplyPoint3x4((Vector3)minNode.position).y, Mathf.Clamp(zf,nz-0.5f,nz+0.5f)+0.5f));
						}
					}
				}
				
				if (minNode != null) {
					if (overlap == 0) {
						nn.node = minNode;
						nn.clampedPosition = clampedPosition;
						return nn;
					}
					else overlap--;
				}
				
				//No nodes were inside grid bounds
				if (!anyInside) {
					nn.node = minNode;
					nn.clampedPosition = clampedPosition;
					return nn;
				}
			}
			//return new NNInfo ();
		}
		
		/** Sets up #neighbourOffsets with the current settings. #neighbourOffsets, #neighbourCosts, #neighbourXOffsets and #neighbourZOffsets are set up.\n
		 * The cost for a non-diagonal movement between two adjacent nodes is RoundToInt (#nodeSize * Int3.Precision)\n
		 * The cost for a diagonal movement between two adjacent nodes is RoundToInt (#nodeSize * Sqrt (2) * Int3.Precision) */
		public virtual void SetUpOffsetsAndCosts () {
#if ASTARDEBUG
			Debug.Log ("+++ --- GridGraph Setting Up Offsets and Costs");
#endif
			
			//First 4 are for the four directly adjacent nodes the last 4 are for the diagonals
			neighbourOffsets[0] = -width;
			neighbourOffsets[1] = 1;
			neighbourOffsets[2] = width;
			neighbourOffsets[3] = -1;
			neighbourOffsets[4] = -width+1;
			neighbourOffsets[5] = width+1;
			neighbourOffsets[6] = width-1;
			neighbourOffsets[7] = -width-1;
			
			uint straightCost = (uint)Mathf.RoundToInt (nodeSize*Int3.Precision);
			uint diagonalCost = (uint)Mathf.RoundToInt (nodeSize*Mathf.Sqrt (2F)*Int3.Precision);
			
			//Diagonals cost more (sqrt(2) times more)
			neighbourCosts[0] = straightCost;
			neighbourCosts[1] = straightCost;
			neighbourCosts[2] = straightCost;
			neighbourCosts[3] = straightCost;
			neighbourCosts[4] = diagonalCost;
			neighbourCosts[5] = diagonalCost;
			neighbourCosts[6] = diagonalCost;
			neighbourCosts[7] = diagonalCost;
			
			
			neighbourXOffsets[0] = 0;
			neighbourXOffsets[1] = 1;
			neighbourXOffsets[2] = 0;
			neighbourXOffsets[3] = -1;
			neighbourXOffsets[4] = 1;
			neighbourXOffsets[5] = 1;
			neighbourXOffsets[6] = -1;
			neighbourXOffsets[7] = -1;
			
			neighbourZOffsets[0] = -1;
			neighbourZOffsets[1] =  0;
			neighbourZOffsets[2] =  1;
			neighbourZOffsets[3] =  0;
			neighbourZOffsets[4] = -1;
			neighbourZOffsets[5] =  1;
			neighbourZOffsets[6] =  1;
			neighbourZOffsets[7] = -1;
		}
		
		public override void ScanInternal (OnScanStatus statusCallback) {
			
			AstarPath.OnPostScan += new OnScanDelegate (OnPostScan);
			
			scans++;
			
			if (nodeSize <= 0) {
				return;
			}
			
			GenerateMatrix ();
			
			if (width > 1024 || depth > 1024) {
				Debug.LogError ("One of the grid's sides is longer than 1024 nodes");
				return;
			}

#if !ASTAR_JPS
			if (this.useJumpPointSearch) {
				Debug.LogError ("Trying to use Jump Point Search, but support for it is not enabled. Please enable it in the inspector (Grid Graph settings).");
			}
#endif
			
			SetUpOffsetsAndCosts ();
			
			int graphIndex = AstarPath.active.astarData.GetGraphIndex(this);
			GridNode.SetGridGraph (graphIndex,this);

			nodes = new GridNode[width*depth];
			for (int i=0;i<nodes.Length;i++) {
				nodes[i] = new GridNode(active);
				nodes[i].GraphIndex = (uint)graphIndex;
			}
			
			if (collision == null) {
				collision = new GraphCollision ();
			}
			collision.Initialize (matrix,nodeSize);
			
			textureData.Initialize ();
			
			for (int z = 0; z < depth; z ++) {
				for (int x = 0; x < width; x++) {
					
					GridNode node = nodes[z*width+x];
					
					node.NodeInGridIndex = z*width+x;
					
					UpdateNodePositionCollision (node,x,z);
					
					textureData.Apply (node,x,z);
				}
			}
			
			
			for (int z = 0; z < depth; z ++) {
				for (int x = 0; x < width; x++) {
				
					GridNode node = nodes[z*width+x];
						
					CalculateConnections (nodes,x,z,node);
					
#if ASTARDEBUG
					if (z == 5 && x == 5) {
						int index = z*width+x;
						Debug.DrawRay ((Vector3)node.position,(Vector3)(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.red);
						Debug.DrawRay ((Vector3)node.position,(Vector3)(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.green);
						Debug.DrawRay ((Vector3)node.position,(Vector3)(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.blue);
						Debug.DrawRay ((Vector3)node.position,(Vector3)(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.yellow);
						Debug.DrawRay ((Vector3)node.position,(Vector3)(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.cyan);
						Debug.DrawRay ((Vector3)node.position,(Vector3)(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.magenta);
						Debug.DrawRay ((Vector3)node.position,(Vector3)(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.black);
						Debug.DrawRay ((Vector3)node.position,(Vector3)(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.white);
					}
#endif
				}
			}
			
			
			ErodeWalkableArea ();
		}
		
		/** Updates position, walkability and penalty for the node.
		 * Assumes that collision.Initialize (...) has been called before this function */
		public virtual void UpdateNodePositionCollision (GridNode node, int x, int z, bool resetPenalty = true) {
			
			node.position = GetNodePosition ( node.NodeInGridIndex, 0 );//0;// = (Int3)matrix.MultiplyPoint3x4 (new Vector3 (x+0.5F,0,z+0.5F));
			
			RaycastHit hit;
			
			bool walkable = true;

			Vector3 position = collision.CheckHeight ((Vector3)node.position, out hit, out walkable);
			node.position = (Int3)position;
			
			if (resetPenalty)
				node.Penalty = initialPenalty;
			
			if (penaltyPosition && resetPenalty) {
				node.Penalty += (uint)Mathf.RoundToInt ((node.position.y-penaltyPositionOffset)*penaltyPositionFactor);
			}

			//Check if the node is on a slope steeper than permitted
			if (walkable && useRaycastNormal && collision.heightCheck) {
				
				if (hit.normal != Vector3.zero) {
					//Take the dot product to find out the cosinus of the angle it has (faster than Vector3.Angle)
					float angle = Vector3.Dot (hit.normal.normalized,collision.up);
					if ( Mathf.Abs ( 1 - collision.up.magnitude ) > 0.1f ) {
						Debug.Log ("HI");
					}

					//Add penalty based on normal
					if (penaltyAngle && resetPenalty) {
						node.Penalty += (uint)Mathf.RoundToInt ((1F-Mathf.Pow(angle, penaltyAnglePower))*penaltyAngleFactor);
					}
					
					//Max slope in cosinus
					float cosAngle = Mathf.Cos (maxSlope*Mathf.Deg2Rad);
					
					//Check if the slope is flat enough to stand on
					if (angle < cosAngle) {
						walkable = false;
					}
				}
			}
			
			//If the walkable flag has already been set to false, there is no point in checking for it again
			if (walkable)
				node.Walkable = collision.Check ((Vector3)node.position);
			else
				node.Walkable = walkable;
			
			node.WalkableErosion = node.Walkable;
		}
		
		/** Erodes the walkable area. \see #erodeIterations */
		public virtual void ErodeWalkableArea () {
			ErodeWalkableArea (0,0,Width,Depth);
		}
		
		/** Erodes the walkable area.
		 * 
		 * xmin, zmin (inclusive)\n
		 * xmax, zmax (exclusive)
		 * 
		 * \see #erodeIterations */
		public virtual void ErodeWalkableArea (int xmin, int zmin, int xmax, int zmax) {
			//Clamp values to grid
			xmin = xmin < 0 ? 0 : (xmin > Width ? Width : xmin);
			xmax = xmax < 0 ? 0 : (xmax > Width ? Width : xmax);
			zmin = zmin < 0 ? 0 : (zmin > Depth ? Depth : zmin);
			zmax = zmax < 0 ? 0 : (zmax > Depth ? Depth : zmax);
			
			if (!erosionUseTags) {
				for (int it=0;it < erodeIterations;it++) {
					for (int z = zmin; z < zmax; z ++) {
						for (int x = xmin; x < xmax; x++) {
							GridNode node = nodes[z*Width+x];
							
							if (!node.Walkable) {
								
								/*int index = node.GetIndex ();
								
								for (int i=0;i<4;i++) {
									if (node.GetConnection (i)) {
										nodes[index+neighbourOffsets[i]].walkable = false;
									}
								}*/
							} else {
								bool anyFalseConnections = false;
							
								for (int i=0;i<4;i++) {
									if (!this.HasNodeConnection (node,i)) {
										anyFalseConnections = true;
										break;
									}
								}
								
								if (anyFalseConnections) {
									node.Walkable = false;
								}
							}
						}
					}
					
					//Recalculate connections
					for (int z = zmin; z < zmax; z ++) {
						for (int x = xmin; x < xmax; x++) {
							GridNode node = nodes[z*Width+x];
							CalculateConnections (nodes,x,z,node);
						}
					}
				}
			} else {
				if (erodeIterations+erosionFirstTag > 31) {
					Debug.LogError ("Too few tags available for "+erodeIterations+" erode iterations and starting with tag " + erosionFirstTag + " (erodeIterations+erosionFirstTag > 31)");
					return;
				}
				if (erosionFirstTag <= 0) {
					Debug.LogError ("First erosion tag must be greater or equal to 1");
					return;
				}
				
				for (int it=0;it < erodeIterations;it++) {
					for (int z = zmin; z < zmax; z ++) {
						for (int x = xmin; x < xmax; x++) {
							GridNode node = nodes[z*width+x] as GridNode;
							
							if (node.Walkable && node.Tag >= erosionFirstTag && node.Tag < erosionFirstTag + it) {
								
								for (int i=0;i<4;i++) {
									GridNode other = GetNodeConnection (node,i);
									if (other != null) {
										uint tag = other.Tag;
										if (tag > erosionFirstTag + it || tag < erosionFirstTag) {
											other.Tag = (uint)(erosionFirstTag+it);
										}
									}
								}
							} else if (node.Walkable && it == 0) {
								bool anyFalseConnections = false;
								
								for (int i=0;i<4;i++) {
									if (!HasNodeConnection (node, i)) {
										anyFalseConnections = true;
										break;
									}
								}
								
								if (anyFalseConnections) {
									node.Tag = (uint)(erosionFirstTag+it);
								}
							}
						}
					}
				}
			}
		}
		
		/** Returns true if a connection between the adjacent nodes \a n1 and \a n2 is valid. Also takes into account if the nodes are walkable */
		public virtual bool IsValidConnection (GridNode n1, GridNode n2) {
			if (!n1.Walkable || !n2.Walkable) {
				return false;
			}
			
			if (maxClimb != 0 && Mathf.Abs (n1.position[maxClimbAxis] - n2.position[maxClimbAxis]) > maxClimb*Int3.Precision) {
				return false;
			}
			
			return true;
		}
		
		/** To reduce memory allocations this array is reused.
		 * Used in the CalculateConnections function
		 * \see CalculateConnections */
		[System.NonSerialized]
		protected int[] corners;
		
		/** Calculates the grid connections for a single node. Convenience function, it's faster to use CalculateConnections(GridNode[],int,int,node) but that will only show when calculating for a large number of nodes
		  * \todo Test this function, should work ok, but you never know */
		public static void CalculateConnections (GridNode node) {
			GridGraph gg = AstarData.GetGraph (node) as GridGraph;
			
			if (gg != null) {
				int index = node.NodeInGridIndex;
				int x = index % gg.width;
				int z = index / gg.width;
				gg.CalculateConnections (gg.nodes,x,z,node);
			}
		}
		
		/** Calculates the grid connections for a single node */
		public virtual void CalculateConnections (GridNode[] nodes, int x, int z, GridNode node) {
			
			//Reset all connections
			// This makes the node have NO connections to any neighbour nodes
			node.ResetConnectionsInternal ();
			
			//All connections are disabled if the node is not walkable
			if (!node.Walkable) {
				return;
			}

			// Internal index of where in the graph the node is
			int index = node.NodeInGridIndex;

			if (corners == null) {
				corners = new int[4];
			} else {
				for (int i = 0;i<4;i++) {
					corners[i] = 0;
				}
			}

			// Loop through axis aligned neighbours (up, down, right, left)
			for (int i=0, j = 3; i<4; j = i, i++) {
				
				int nx = x + neighbourXOffsets[i];
				int nz = z + neighbourZOffsets[i];
				
				if (nx < 0 || nz < 0 || nx >= width || nz >= depth) {
					continue;
				}
				
				GridNode other = nodes[index+neighbourOffsets[i]] as GridNode;
				
				if (IsValidConnection (node, other)) {
					node.SetConnectionInternal (i, true);
					corners[i]++;
					corners[j]++;
				} else {
					node.SetConnectionInternal (i, false);
				}
			}

			// Add in the diagonal connections
			if (neighbours == NumNeighbours.Eight) {
				if (cutCorners) {
					for (int i=0; i<4; i++) {
						
						if (corners[i] >= 1) {
							int nx = x + neighbourXOffsets[i+4];
							int nz = z + neighbourZOffsets[i+4];
						
							if (nx < 0 || nz < 0 || nx >= width || nz >= depth) {
								continue;
							}
					
							GridNode other = nodes[index+neighbourOffsets[i+4]];
							
							node.SetConnectionInternal (i+4, IsValidConnection (node,other));
						}
					}
				} else {
					for (int i=0; i<4; i++) {
						
						//We don't need to check if it is out of bounds because if both of the other neighbours are inside the bounds this one must be too
						if (corners[i] == 2) {
							GridNode other = nodes[index+neighbourOffsets[i+4]];
							
							node.SetConnectionInternal (i+4, IsValidConnection (node,other));
						}
					}
				}
			}
			
		}
		
		/** Auto links grid graphs together. Called after all graphs have been scanned.
		 * \see autoLinkGrids
		 */
		public void OnPostScan (AstarPath script) {
			
			AstarPath.OnPostScan -= new OnScanDelegate (OnPostScan);
			
			if (!autoLinkGrids || autoLinkDistLimit <= 0) {
				return;
			}
			
			//Link to other grids
			
			throw new System.NotSupportedException ();
			
#if FALSE
			
			int maxCost = Mathf.RoundToInt (autoLinkDistLimit * Int3.Precision);
			
			//Loop through all GridGraphs
			foreach (GridGraph gg in script.astarData.FindGraphsOfType (typeof (GridGraph))) {
				
				if (gg == this || gg.nodes == null || nodes == null) {
					continue;
				}
				
				//Int3 prevPos = gg.GetNearest (nodes[0]).position;
				
				//Z = 0
				for (int x = 0; x < width;x++) {
					
					GraphNode node1 = nodes[x];
					GraphNode node2 = gg.GetNearest ((Vector3)node1.Position).node;
					
					Vector3 pos = inverseMatrix.MultiplyPoint3x4 ((Vector3)node2.Position);
					
					if (pos.z > 0) {
						continue;
					}
					
					int cost = (node1.Position-node2.Position).costMagnitude;
					
					if (cost > maxCost) {
						continue;
					}
					
					node1.AddConnection (node2,cost);
					node2.AddConnection (node1,cost);
				}
				
				//X = 0
				for (int z = 0; z < depth;z++) {
					
					GraphNode node1 = nodes[z*width];
					GraphNode node2 = gg.GetNearest ((Vector3)node1.Position).node;
					
					Vector3 pos = inverseMatrix.MultiplyPoint3x4 ((Vector3)node2.Position);
					
					if (pos.x > 0) {
						continue;
					}
					
					int cost = (node1.Position-node2.Position).costMagnitude;
					
					if (cost > maxCost) {
						continue;
					}
					
					node1.AddConnection (node2,cost);
					node2.AddConnection (node1,cost);
				}
				
				//Z = max
				for (int x = 0; x < width;x++) {
					
					GraphNode node1 = nodes[(depth-1)*width+x];
					GraphNode node2 = gg.GetNearest ((Vector3)node1.Position).node;
					
					Vector3 pos = inverseMatrix.MultiplyPoint3x4 ((Vector3)node2.Position);
					
					if (pos.z < depth-1) {
						continue;
					}
					
					//Debug.DrawLine (node1.position,node2.position,Color.red);
					int cost = (node1.Position-node2.Position).costMagnitude;
					
					if (cost > maxCost) {
						continue;
					}
					
					node1.AddConnection (node2,cost);
					node2.AddConnection (node1,cost);
				}
				
				//X = max
				for (int z = 0; z < depth;z++) {
					
					GraphNode node1 = nodes[z*width+width-1];
					GraphNode node2 = gg.GetNearest ((Vector3)node1.Position).node;
					
					Vector3 pos = inverseMatrix.MultiplyPoint3x4 ((Vector3)node2.Position);
					
					if (pos.x < width-1) {
						continue;
					}
					
					int cost = (node1.Position-node2.Position).costMagnitude;
					
					if (cost > maxCost) {
						continue;
					}
					
					
					
					node1.AddConnection (node2,cost);
					node2.AddConnection (node1,cost);
				}
			}
#endif
	
		}
		
		public override void OnDrawGizmos (bool drawNodes) {

			Gizmos.matrix = boundsMatrix;
			Gizmos.color = Color.white;
			Gizmos.DrawWireCube (Vector3.zero, new Vector3 (size.x,0,size.y));
			
			Gizmos.matrix = Matrix4x4.identity;
			
			if (!drawNodes) {
				return;
			}
			
			if (nodes == null || depth*width != nodes.Length) {
				//Scan (AstarPath.active.GetGraphIndex (this));
				return;
			}

			PathHandler debugData = AstarPath.active.debugPathData;
			
			GridNode node = null;

			for (int z = 0; z < depth; z ++) {
				for (int x = 0; x < width; x++) {
					node = nodes[z*width+x] as GridNode;
					
					if (!node.Walkable) {
						continue;
					}

					Gizmos.color = NodeColor (node,AstarPath.active.debugPathData);

					if (AstarPath.active.showSearchTree && debugData != null) {
						if (InSearchTree(node,AstarPath.active.debugPath)) {
							PathNode nodeR = debugData.GetPathNode (node);
							if (nodeR != null && nodeR.parent != null) {
								Gizmos.DrawLine ((Vector3)node.position, (Vector3)nodeR.parent.node.position);
							}
						}
					} else {
						for (int i=0;i<8;i++) {
							GridNode other = GetNodeConnection (node, i);
							if (other != null) {
								Gizmos.DrawLine ((Vector3)node.position, (Vector3)other.position);
							}
						}
#if ASTAR_GRID_CUSTOM_CONNECTIONS
						if ( node.connections != null ) for (int i=0;i<node.connections.Length;i++) {
							GraphNode other = node.connections[i];
							Gizmos.DrawLine ((Vector3)node.position, (Vector3)other.position);
						}
#endif
					}
					
				}
			}
		}
		
		/** Calculates minimum and maximum points for bounds \a b when multiplied with the matrix */
		public void GetBoundsMinMax (Bounds b, Matrix4x4 matrix, out Vector3 min, out Vector3 max) {
			Vector3[] p = new Vector3[8];
			
			p[0] = matrix.MultiplyPoint3x4 (b.center + new Vector3 ( b.extents.x, b.extents.y, b.extents.z));
			p[1] = matrix.MultiplyPoint3x4 (b.center + new Vector3 ( b.extents.x, b.extents.y,-b.extents.z));
			p[2] = matrix.MultiplyPoint3x4 (b.center + new Vector3 ( b.extents.x,-b.extents.y, b.extents.z));
			p[3] = matrix.MultiplyPoint3x4 (b.center + new Vector3 ( b.extents.x,-b.extents.y,-b.extents.z));
			p[4] = matrix.MultiplyPoint3x4 (b.center + new Vector3 (-b.extents.x, b.extents.y, b.extents.z));
			p[5] = matrix.MultiplyPoint3x4 (b.center + new Vector3 (-b.extents.x, b.extents.y,-b.extents.z));
			p[6] = matrix.MultiplyPoint3x4 (b.center + new Vector3 (-b.extents.x,-b.extents.y, b.extents.z));
			p[7] = matrix.MultiplyPoint3x4 (b.center + new Vector3 (-b.extents.x,-b.extents.y,-b.extents.z));
			
			min = p[0];
			max = p[0];
			for (int i=1;i<8;i++) {
				min = Vector3.Min (min,p[i]);
				max = Vector3.Max (max,p[i]);
			}
		}
		
		/** All nodes inside the bounding box.
		  * \note Be nice to the garbage collector and release the list when you have used it (optional)
		  * \see Pathfinding.Util.ListPool
		  * 
		  * \see GetNodesInArea(GraphUpdateShape)
		  */
		public List<GraphNode> GetNodesInArea (Bounds b) {
			return GetNodesInArea (b, null);
		}
		
		/** All nodes inside the shape.
		  * \note Be nice to the garbage collector and release the list when you have used it (optional)
		  * \see Pathfinding.Util.ListPool
		  * 
		  * \see GetNodesInArea(Bounds)
		  */
		public List<GraphNode> GetNodesInArea (GraphUpdateShape shape) {
			return GetNodesInArea (shape.GetBounds (), shape);
		}
		
		/** All nodes inside the shape or if null, the bounding box.
		 * If a shape is supplied, it is assumed to be contained inside the bounding box.
		 * \see GraphUpdateShape.GetBounds
		 */
		private List<GraphNode> GetNodesInArea (Bounds b, GraphUpdateShape shape) {
			
			if (nodes == null || width*depth != nodes.Length) {
				return null;
			}
			
			List<GraphNode> inArea = Pathfinding.Util.ListPool<GraphNode>.Claim ();
			
			Vector3 min, max;
			GetBoundsMinMax (b,inverseMatrix,out min, out max);
			
			int minX = Mathf.RoundToInt (min.x-0.5F);
			int maxX = Mathf.RoundToInt (max.x-0.5F);
			
			int minZ = Mathf.RoundToInt (min.z-0.5F);
			int maxZ = Mathf.RoundToInt (max.z-0.5F);
			
			IntRect originalRect = new IntRect(minX,minZ,maxX,maxZ);
			IntRect gridRect = new IntRect(0,0,width-1,depth-1);
			
			IntRect rect = IntRect.Intersection (originalRect, gridRect);
			
			for (int x = rect.xmin; x <= rect.xmax;x++) {
				for (int z = rect.ymin;z <= rect.ymax;z++) {
					
					int index = z*width+x;
					
					GraphNode node = nodes[index];
					
					if (b.Contains ((Vector3)node.position) && (shape == null || shape.Contains ((Vector3)node.position))) {
						inArea.Add (node);
					}
				}
			}
			
			return inArea;
		}
		
		public GraphUpdateThreading CanUpdateAsync (GraphUpdateObject o) {
			return GraphUpdateThreading.UnityThread;
		}
		
		public void UpdateAreaInit (GraphUpdateObject o) {}
		
		/** Internal function to update an area of the graph.
		  */
		public void UpdateArea (GraphUpdateObject o) {
			
			if (nodes == null || nodes.Length != width*depth) {
				Debug.LogWarning ("The Grid Graph is not scanned, cannot update area ");
				//Not scanned
				return;
			}
			
			//Copy the bounds
			Bounds b = o.bounds;
			
			Vector3 min, max;
			GetBoundsMinMax (b,inverseMatrix,out min, out max);
			
			int minX = Mathf.RoundToInt (min.x-0.5F);
			int maxX = Mathf.RoundToInt (max.x-0.5F);
			
			int minZ = Mathf.RoundToInt (min.z-0.5F);
			int maxZ = Mathf.RoundToInt (max.z-0.5F);
			//We now have coordinates in local space (i.e 1 unit = 1 node)
			
			IntRect originalRect = new IntRect(minX,minZ,maxX,maxZ);
			IntRect affectRect = originalRect;
			
			IntRect gridRect = new IntRect(0,0,width-1,depth-1);
			
			IntRect physicsRect = originalRect;
			
			int erosion = o.updateErosion ? erodeIterations : 0;
			
#if ASTARDEBUG
			Matrix4x4 debugMatrix = matrix;
			debugMatrix *= Matrix4x4.TRS (new Vector3(0.5f,0,0.5f),Quaternion.identity,Vector3.one);
			
			originalRect.DebugDraw (debugMatrix,Color.red);
#endif
			
			bool willChangeWalkability = o.updatePhysics || o.modifyWalkability;
			
			//Calculate the largest bounding box which might be affected
			
			if (o.updatePhysics && !o.modifyWalkability) {
				//Add the collision.diameter margin for physics calls
				if (collision.collisionCheck) {
					Vector3 margin = new Vector3 (collision.diameter,0,collision.diameter)*0.5F;
					
					min -= margin*1.02F;//0.02 safety margin, physics is rarely very accurate
					max += margin*1.02F;
					
					physicsRect = new IntRect(
					                            Mathf.RoundToInt (min.x-0.5F),
					                            Mathf.RoundToInt (min.z-0.5F),
					                            Mathf.RoundToInt (max.x-0.5F),
					                            Mathf.RoundToInt (max.z-0.5F)
					                            );
					
					affectRect = IntRect.Union (physicsRect, affectRect);
				}
			}
			
			if (willChangeWalkability || erosion > 0) {
				//Add affect radius for erosion. +1 for updating connectivity info at the border
				affectRect = affectRect.Expand (erosion + 1);
			}
			
			IntRect clampedRect = IntRect.Intersection (affectRect,gridRect);
			
			//Mark nodes that might be changed
			for (int x = clampedRect.xmin; x <= clampedRect.xmax;x++) {
				for (int z = clampedRect.ymin;z <= clampedRect.ymax;z++) {
					o.WillUpdateNode (nodes[z*width+x]);
				}
			}
			
			//Update Physics
			if (o.updatePhysics && !o.modifyWalkability) {
				
				collision.Initialize (matrix,nodeSize);
				
				clampedRect = IntRect.Intersection (physicsRect,gridRect);
				
				for (int x = clampedRect.xmin; x <= clampedRect.xmax;x++) {
					for (int z = clampedRect.ymin;z <= clampedRect.ymax;z++) {
						
						int index = z*width+x;
						
						GridNode node = nodes[index];
						
						UpdateNodePositionCollision (node,x,z, o.resetPenaltyOnPhysics);
					}
				}
			}
			
			//Apply GUO
			
			clampedRect = IntRect.Intersection (originalRect, gridRect);
			for (int x = clampedRect.xmin; x <= clampedRect.xmax;x++) {
				for (int z = clampedRect.ymin;z <= clampedRect.ymax;z++) {
					int index = z*width+x;
					
					GridNode node = nodes[index];
					
					if (willChangeWalkability) {
						node.Walkable = node.WalkableErosion;
						if (o.bounds.Contains ((Vector3)node.position)) o.Apply (node);
						node.WalkableErosion = node.Walkable;
					} else {
						if (o.bounds.Contains ((Vector3)node.position)) o.Apply (node);
					}
				}
			}
		
#if ASTARDEBUG
			physicsRect.DebugDraw (debugMatrix,Color.blue);
			affectRect.DebugDraw (debugMatrix,Color.black);
#endif
			
			//Recalculate connections
			if (willChangeWalkability && erosion == 0) {
				
				clampedRect = IntRect.Intersection (affectRect, gridRect);
				for (int x = clampedRect.xmin; x <= clampedRect.xmax;x++) {
					for (int z = clampedRect.ymin;z <= clampedRect.ymax;z++) {
						int index = z*width+x;
						
						GridNode node = nodes[index];
						
						CalculateConnections (nodes,x,z,node);
					}
				}
			} else if (willChangeWalkability && erosion > 0) {
				
				
				clampedRect = IntRect.Union (originalRect, physicsRect);
				
				IntRect erosionRect1 = clampedRect.Expand (erosion);
				IntRect erosionRect2 = erosionRect1.Expand (erosion);
				
				erosionRect1 = IntRect.Intersection (erosionRect1,gridRect);
				erosionRect2 = IntRect.Intersection (erosionRect2,gridRect);
				
#if ASTARDEBUG
				erosionRect1.DebugDraw (debugMatrix,Color.magenta);
				erosionRect2.DebugDraw (debugMatrix,Color.cyan);
#endif
				
				/*
				all nodes inside clampedRect might have had their walkability changed
				all nodes inside erosionRect1 might get affected by erosion from clampedRect and erosionRect2
				all nodes inside erosionRect2 (but outside erosionRect1) will be reset to previous walkability
				after calculation since their erosion might not be correctly calculated (nodes outside erosionRect2 would maybe have effect)
				*/
				
				for (int x = erosionRect2.xmin; x <= erosionRect2.xmax;x++) {
					for (int z = erosionRect2.ymin;z <= erosionRect2.ymax;z++) {
						
						int index = z*width+x;
						
						GridNode node = nodes[index];
						
						bool tmp = node.Walkable;
						node.Walkable = node.WalkableErosion;
						
						if (!erosionRect1.Contains (x,z)) {
							//Save the border's walkabilty data (will be reset later)
							node.TmpWalkable = tmp;
						}
					}
				}
				
				for (int x = erosionRect2.xmin; x <= erosionRect2.xmax;x++) {
					for (int z = erosionRect2.ymin;z <= erosionRect2.ymax;z++) {
						int index = z*width+x;
						
						GridNode node = nodes[index];
						
#if ASTARDEBUG
						if (!node.Walkable)
							Debug.DrawRay ((Vector3)node.position, Vector3.up*2,Color.red);
#endif
						
						CalculateConnections (nodes,x,z,node);
					}
				}
				
				//Erode the walkable area
				ErodeWalkableArea (erosionRect2.xmin,erosionRect2.ymin,erosionRect2.xmax+1,erosionRect2.ymax+1);
				
				for (int x = erosionRect2.xmin; x <= erosionRect2.xmax;x++) {
					for (int z = erosionRect2.ymin;z <= erosionRect2.ymax;z++) {
						if (erosionRect1.Contains (x,z)) continue;
						
						int index = z*width+x;
						
						GridNode node = nodes[index];
						
						//Restore temporarily stored data
						node.Walkable = node.TmpWalkable;
					}
				}
				
				//Recalculate connections of all affected nodes
				for (int x = erosionRect2.xmin; x <= erosionRect2.xmax;x++) {
					for (int z = erosionRect2.ymin;z <= erosionRect2.ymax;z++) {
						int index = z*width+x;
						
						GridNode node = nodes[index];
						CalculateConnections (nodes,x,z,node);
					}
				}
			}
		}
		
		/** Returns if there is an obstacle between \a origin and \a end on the graph.
		 * This is not the same as Physics.Linecast, this function traverses the graph and looks for collisions.
		 * \astarpro */
		public bool Linecast (Vector3 _a, Vector3 _b) {
			GraphHitInfo hit;
			return Linecast (_a,_b,null, out hit);
		}
		
		/** Returns if there is an obstacle between \a origin and \a end on the graph.
		 * \param [in] _a Point to linecast from
		 * \param [in] _b Point to linecast to
		 * \param [in] hint If you have some idea of what the start node might be (the one close to \a _a), pass it to hint since it can enable faster lookups
		 * This is not the same as Physics.Linecast, this function traverses the graph and looks for collisions.
		 * \astarpro */
		public bool Linecast (Vector3 _a, Vector3 _b, GraphNode hint) {
			GraphHitInfo hit;
			return Linecast (_a,_b,hint, out hit);
		}
		
		/** Returns if there is an obstacle between \a origin and \a end on the graph.
		 * \param [in] _a Point to linecast from
		 * \param [in] _b Point to linecast to
		 * \param [out] hit Contains info on what was hit, see GraphHitInfo
		 * \param [in] hint If you have some idea of what the start node might be (the one close to \a _a), pass it to hint since it can enable faster lookups
		 * This is not the same as Physics.Linecast, this function traverses the graph and looks for collisions.
		 * \astarpro */
		public bool Linecast (Vector3 _a, Vector3 _b, GraphNode hint, out GraphHitInfo hit) {
			return Linecast (_a, _b, hint, out hit, null);
		}
		
		/** Returns if there is an obstacle between \a origin and \a end on the graph.
		 * \param [in] _a Point to linecast from
		 * \param [in] _b Point to linecast to
		 * \param [out] hit Contains info on what was hit, see GraphHitInfo
		 * \param [in] hint If you have some idea of what the start node might be (the one close to \a _a), pass it to hint since it can enable faster lookups
		 * \param trace If a list is passed, then it will be filled with all nodes the linecast traverses
		 * This is not the same as Physics.Linecast, this function traverses the graph and looks for collisions.
		 * \astarpro */
		public bool Linecast (Vector3 _a, Vector3 _b, GraphNode hint, out GraphHitInfo hit, List<GraphNode> trace) {
			hit = new GraphHitInfo ();
			
			//
			//Node n2 = GetNearest (_b,NNConstraint.None);
			
			_a = inverseMatrix.MultiplyPoint3x4 (_a);
			_a.x -= 0.5F;
			_a.z -= 0.5F;
			
			_b = inverseMatrix.MultiplyPoint3x4 (_b);
			_b.x -= 0.5F;
			_b.z -= 0.5F;
			
			//Grid coordinates
			//Int3 a = new Int3 (Mathf.RoundToInt (_a.x),Mathf.RoundToInt (_a.y),Mathf.RoundToInt (_a.z));
			//Int3 b = new Int3 (Mathf.RoundToInt (_b.x),Mathf.RoundToInt (_b.y),Mathf.RoundToInt (_b.z));
			
			//Clamping is needed
			if (_a.x < -0.5F || _a.z < -0.5F || _a.x >= width-0.5F || _a.z >= depth-0.5F || 
			    _b.x < -0.5F || _b.z < -0.5F || _b.x >= width-0.5F || _b.z >= depth-0.5F) {
				
				//Bounding points of the grid
				Vector3 p1 = new Vector3 (-0.5F     ,0,      -0.5F);
				Vector3 p2 = new Vector3 (-0.5F     ,0,	depth-0.5F);
				Vector3 p3 = new Vector3 (width-0.5F,0,	depth-0.5F);
				Vector3 p4 = new Vector3 (width-0.5F,0,		 -0.5F);
				
				int intersectCount = 0;
				
				bool intersect = false;
				Vector3 intersection = Polygon.SegmentIntersectionPoint (p1,p2,_a,_b, out intersect);
				
				if (intersect) {
					//Debug.Log ("Intersection with p1 and p2 "+_a+" "+_b+" - Intersection: "+intersection);
					intersectCount++;
					if (!Polygon.Left (p1,p2,_a)) {
						_a = intersection;
					} else {
						_b = intersection;
					}
				}
				intersection = Polygon.SegmentIntersectionPoint (p2,p3,_a,_b, out intersect);
				
				if (intersect) {
					//Debug.Log ("Intersection with p2 and p3 "+_a+" "+_b+" - Intersection: "+intersection);
					intersectCount++;
					if (!Polygon.Left (p2,p3,_a)) {
						_a = intersection;
					} else {
						_b = intersection;
					}
				}
				intersection = Polygon.SegmentIntersectionPoint (p3,p4,_a,_b, out intersect);
				
				if (intersect) {
					//Debug.Log ("Intersection with p3 and p4 "+_a+" "+_b+" - Intersection: "+intersection);
					intersectCount++;
					if (!Polygon.Left (p3,p4,_a)) {
						_a = intersection;
					} else {
						_b = intersection;
					}
				}
				intersection = Polygon.SegmentIntersectionPoint (p4,p1,_a,_b, out intersect);
				
				if (intersect) {
					//Debug.Log ("Intersection with p4 and p1 "+_a+" "+_b+" - Intersection: "+intersection);
					intersectCount++;
					if (!Polygon.Left (p4,p1,_a)) {
						_a = intersection;
					} else {
						_b = intersection;
					}
				}
				
				if (intersectCount == 0) {
					//The line does not intersect with the grid
					return false;
				}
			}
			
			Vector3 dir = _b-_a;
			float magn = dir.magnitude;
			
			if (magn == 0) {
				//Zero length line
				return false;
			}
			
			float sampleLength = 0.2F;
			
			float newMagn = nodeSize * sampleLength;
			newMagn -= nodeSize * 0.02F;
			
			dir = (dir / magn) * newMagn;
			
			//Floor to int, number of samples on the line
			int its = (int)(magn / newMagn);
			
			Vector3 originOffset = _a + dir * nodeSize * 0.01F;
			
			GraphNode prevNode = null;
			
			for (int i=0;i <= its;i++) {
				
				Vector3 p = originOffset + dir * i;
				
				int x = Mathf.RoundToInt (p.x);
				int z = Mathf.RoundToInt (p.z);
				
				x = x < 0 ? 0 : (x >= width ? width-1 : x);
				z = z < 0 ? 0 : (z >= depth ? depth-1 : z);
				
				GraphNode node = nodes[z*width+x];
				
				if (node == prevNode) continue;
				
				if (!node.Walkable) {
					if (i > 0) {
						hit.point = matrix.MultiplyPoint3x4 (originOffset + dir * (i-1)+new Vector3 (0.5F,0,0.5F));
					} else {
						hit.point = matrix.MultiplyPoint3x4 (_a+new Vector3 (0.5F,0,0.5F));
					}
					hit.origin = matrix.MultiplyPoint3x4 (_a+new Vector3 (0.5F,0,0.5F));
					hit.node = node;
					return true;
				}
				
				if (i > its-1) {
					if (Mathf.Abs (p.x-_b.x) <= 0.50001F || Mathf.Abs (p.z - _b.z) <= 0.50001F) {
						return false;
					}
				}
				
				if (trace != null) trace.Add (node);
				
				prevNode = node;
					
			}
			
			return false;
		}
		
		/** Returns if \a _b is visible from \a _a on the graph.
		 * This function is different from the other Linecast functions since it 1) snaps the start and end positions directly to the graph
		 * and it uses Bresenham's line drawing algorithm as opposed to the others which use sampling at fixed intervals.
		 * If you only care about if one \b node can see another \b node, then this function is great, but if you need more precision than one node,
		 * use the normal linecast functions
		 * 
		 * \param [in] _a Point to linecast from
		 * \param [in] _b Point to linecast to
		 * \param [out] hit Contains info on what was hit, see GraphHitInfo
		 * \param [in] hint If you have some idea of what the start node might be (the one close to \a _a), pass it to hint since it can enable faster lookups
		 * 
		 * This is not the same as Physics.Linecast, this function traverses the graph and looks for collisions.
		 * \astarpro */
		public bool SnappedLinecast (Vector3 _a, Vector3 _b, GraphNode hint, out GraphHitInfo hit) {
			hit = new GraphHitInfo ();
			
			//System.DateTime startTime = System.DateTime.UtcNow;
			
			GraphNode n1 = GetNearest (_a,NNConstraint.None).node;
			GraphNode n2 = GetNearest (_b,NNConstraint.None).node;
			
			_a = inverseMatrix.MultiplyPoint3x4 ((Vector3)n1.position);
			_a.x -= 0.5F;
			_a.z -= 0.5F;
			
			_b = inverseMatrix.MultiplyPoint3x4 ((Vector3)n2.position);
			_b.x -= 0.5F;
			_b.z -= 0.5F;
			
			Int3 a = new Int3 (Mathf.RoundToInt (_a.x),Mathf.RoundToInt (_a.y),Mathf.RoundToInt (_a.z));
			Int3 b = new Int3 (Mathf.RoundToInt (_b.x),Mathf.RoundToInt (_b.y),Mathf.RoundToInt (_b.z));
			
			hit.origin = (Vector3)a;
			
			//Debug.DrawLine (matrix.MultiplyPoint3x4 (a*100),matrix.MultiplyPoint3x4 (b*100),Color.yellow);
			
			if (!nodes[a.z*width+a.x].Walkable) {
				hit.node = nodes[a.z*width+a.x];
				hit.point = matrix.MultiplyPoint3x4 (new Vector3 (a.x+0.5F,0,a.z+0.5F));
				hit.point.y = ((Vector3)hit.node.position).y;
				return true;
			}
			
			int dx = Mathf.Abs (a.x-b.x);
			int dz = Mathf.Abs (a.z-b.z);
			
			int sx = 0;
			int sz = 0;
			
			if (a.x < b.x) {
				sx = 1;
			} else {
				sx = -1;
			}
			
			if (a.z < b.z) {
				sz = 1;
			} else {
				sz = -1;
			}
			
			int err = dx-dz;
			
			while (true) {
				
				if (a.x == b.x && a.z == b.z) {
					return false;
				}
				
				int e2 = err*2;
				
				int dir = 0;
				
				Int3 newPos = a;
				
				if (e2 > -dz) {
					err = err-dz;
					dir = sx;
					newPos.x += sx;
				}
				
				if (e2 < dx) {
					err = err+dx;
					dir += width*sz;
					newPos.z += sz;
				}
				
				if (dir == 0) {
					Debug.LogError ("Offset is zero, this should not happen");
					return false;
				}
				
				for (int i=0;i<neighbourOffsets.Length;i++) {
					if (neighbourOffsets[i] == dir) {
						if (CheckConnection (nodes[a.z*width+a.x] as GridNode,i)) {
							if (!nodes[newPos.z*width+newPos.x].Walkable) {
								hit.node = nodes[a.z*width+a.x];
								hit.point = matrix.MultiplyPoint3x4 (new Vector3 (a.x+0.5F,0,a.z+0.5F));
								hit.point.y = ((Vector3)hit.node.position).y;
								return true;
							}
							
							//Debug.DrawLine (matrix.MultiplyPoint3x4 (a*100),matrix.MultiplyPoint3x4 (newPos*100));
							a = newPos;
							break;
						} else {
						
							hit.node = nodes[a.z*width+a.x];
							hit.point = matrix.MultiplyPoint3x4 (new Vector3 (a.x+0.5F,0,a.z+0.5F));
							hit.point.y = ((Vector3)hit.node.position).y;
							return true;
						}
					}
				}
			}
			
		}
		
		/** Returns if \a node is connected to it's neighbour in the specified direction.
		  * This will also return true if #neighbours = NumNeighbours.Four, the direction is diagonal and one can move through one of the adjacent nodes
		  * to the targeted node.
		  */
		public bool CheckConnection (GridNode node, int dir) {
			if (neighbours == NumNeighbours.Eight) {
				return HasNodeConnection (node, dir);
			} else {
				int dir1 = (dir-4-1) & 0x3;
				int dir2 = (dir-4+1) & 0x3;
				
				if (!HasNodeConnection (node, dir1) || !HasNodeConnection (node, dir2)) {
					return false;
				} else {
					GridNode n1 = nodes[node.NodeInGridIndex+neighbourOffsets[dir1]];
					GridNode n2 = nodes[node.NodeInGridIndex+neighbourOffsets[dir2]];
					
					if (!n1.Walkable || !n2.Walkable) {
						return false;
					}
					
					if (!HasNodeConnection (n2, dir1) || !HasNodeConnection (n1, dir2)) {
						return false;
					}
				}
				return true;
			}
		}

		public override void SerializeExtraInfo (GraphSerializationContext ctx)
		{
			if (nodes == null) {
				ctx.writer.Write(-1);
				return;
			}
			
			ctx.writer.Write (nodes.Length);
			
			for (int i=0;i<nodes.Length;i++) {
				nodes[i].SerializeNode(ctx);
			}
		}
		
		public override void DeserializeExtraInfo (GraphSerializationContext ctx)
		{
			
			int count = ctx.reader.ReadInt32();
			if (count == -1) {
				nodes = null;
				return;
			}
			
			nodes = new GridNode[count];
			
			for (int i=0;i<nodes.Length;i++) {
				nodes[i] = new GridNode (active);
				nodes[i].DeserializeNode(ctx);
			}
		}

#if ASTAR_NO_JSON

		public override void SerializeSettings ( GraphSerializationContext ctx ) {

			base.SerializeSettings (ctx);
			ctx.writer.Write (aspectRatio);
			ctx.SerializeVector3 (rotation);
			ctx.SerializeVector3 (center);
			ctx.SerializeVector3 ((Vector3)unclampedSize);
			ctx.writer.Write (nodeSize);
			// collision
			collision.SerializeSettings (ctx);

			ctx.writer.Write (maxClimb);
			ctx.writer.Write (maxClimbAxis);
			ctx.writer.Write (maxSlope);
			ctx.writer.Write (erodeIterations);
			ctx.writer.Write (erosionUseTags);
			ctx.writer.Write (erosionFirstTag);
			ctx.writer.Write (autoLinkGrids);
			ctx.writer.Write ((int)neighbours);
			ctx.writer.Write (cutCorners);
			ctx.writer.Write (penaltyPosition);
			ctx.writer.Write (penaltyPositionFactor);
			ctx.writer.Write (penaltyAngle);
			ctx.writer.Write (penaltyAngleFactor);
			ctx.writer.Write (penaltyAnglePower);
			ctx.writer.Write (isometricAngle);

			ctx.writer.Write (useJumpPointSearch);
		}
		
		public override void DeserializeSettings ( GraphSerializationContext ctx ) {
			
			base.DeserializeSettings (ctx);
			
			aspectRatio = ctx.reader.ReadSingle();
			rotation = ctx.DeserializeVector3();
			center = ctx.DeserializeVector3();
			unclampedSize = (Vector2)ctx.DeserializeVector3();
			nodeSize = ctx.reader.ReadSingle();
			collision.DeserializeSettings(ctx);
			maxClimb = ctx.reader.ReadSingle();
			maxClimbAxis = ctx.reader.ReadInt32();
			maxSlope = ctx.reader.ReadSingle();
			erodeIterations = ctx.reader.ReadInt32();
			erosionUseTags = ctx.reader.ReadBoolean();
			erosionFirstTag = ctx.reader.ReadInt32();
			autoLinkGrids = ctx.reader.ReadBoolean();
			neighbours = (NumNeighbours)ctx.reader.ReadInt32();
			cutCorners = ctx.reader.ReadBoolean();
			penaltyPosition = ctx.reader.ReadBoolean();
			penaltyPositionFactor = ctx.reader.ReadSingle();
			penaltyAngle = ctx.reader.ReadBoolean();
			penaltyAngleFactor = ctx.reader.ReadSingle();
			penaltyAnglePower = ctx.reader.ReadSingle();
			isometricAngle = ctx.reader.ReadSingle();

			useJumpPointSearch = ctx.reader.ReadBoolean();
		}
#endif

		public override void PostDeserialization () {
			
#if ASTARDEBUG
			Debug.Log ("Grid Graph - Post Deserialize");
#endif
			
			GenerateMatrix ();
			
			SetUpOffsetsAndCosts ();
			
			if (nodes == null || nodes.Length == 0) return;
			
			if (width*depth != nodes.Length) {
				Debug.LogWarning ("Node data did not match with bounds data. Probably a change to the bounds/width/depth data was made after scanning the graph just prior to saving it. Nodes will be discarded");
				nodes = new GridNode[0];
				return;
			}
			
			GridNode.SetGridGraph (AstarPath.active.astarData.GetGraphIndex(this),this);
			
			for (int z = 0; z < depth; z ++) {
				for (int x = 0; x < width; x++) {
					
					GridNode node = nodes[z*width+x] as GridNode;
					
					if (node == null) {
						Debug.LogError ("Deserialization Error : Couldn't cast the node to the appropriate type - GridGenerator. Check the CreateNodes function");
						return;
					}
					
					node.NodeInGridIndex = z*width+x;
				}
			}
		}
	}
	
	public enum NumNeighbours {
		Four,
		Eight
	}
}