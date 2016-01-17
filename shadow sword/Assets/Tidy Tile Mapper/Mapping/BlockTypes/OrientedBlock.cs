using System;
using UnityEngine;
using DopplerInteractive.TidyTileMapper.Utilities;

public class OrientedBlock : Block
{	
	public int orientation;
	
	public float x_offset = 0.0f;
	public float y_offset = 0.0f;
	public float z_offset = 0.0f;
	
	public float x_rotation = 0.0f;
	public float y_rotation = 0.0f;
	public float z_rotation = 0.0f;
		
	public BlockOrientation GetOrientation(){
		
		return (BlockOrientation)orientation;
		
	}
	
	public BlockSet Empty;
	public BlockSet Bottom_Left;
	public BlockSet Bottom_Alone;
	public BlockSet Bottom_Right;
	public BlockSet Bottom_Surrounded;
	public BlockSet Right_Alone;
	public BlockSet Centre_Alone;
	public BlockSet Left_Alone;
	public BlockSet Centre_Surrounded;
	public BlockSet Top_Right;
	public BlockSet Top_Alone;
	public BlockSet Top_Left;
	public BlockSet Top_Surrounded;
	public BlockSet Right_Surrounded;
	public BlockSet Left_Surrounded;
	public BlockSet Centre_Horizontal_Surrounded;
	public BlockSet Centre_Vertical_Surrounded;
	
	public BlockSet Top_Right_Inner_Corner;
	public BlockSet Top_Left_Inner_Corner;
	public BlockSet Bottom_Right_Inner_Corner;
	public BlockSet Bottom_Left_Inner_Corner;
	public BlockSet Right_Diagonal;
	public BlockSet Left_Diagonal;
	public BlockSet Lower_Right_Filled_Diagonal;
	public BlockSet Lower_Left_Filled_Diagonal;
	public BlockSet Upper_Right_Filled_Diagonal;
	public BlockSet Upper_Left_Filled_Diagonal;
	
	public BlockSet Cross_Hub;
	public BlockSet Downward_T_Hub;
	public BlockSet Upward_T_Hub;
	public BlockSet Leftward_T_Hub;
	public BlockSet Rightward_T_Hub;
	
	//New Block Sets
	//Icons created by Alex Evangelou of Angry Fish Studios
	//http://www.angryfishstudios.com/
	//
	//Requested in like... April of 2012 but I didn't get around to it until the 2nd of January 2013
	//Mega-appalling on my part.
	
	public BlockSet Border_Bottom_Left;
	public BlockSet Border_Bottom_Right;
	public BlockSet Border_Top_Left;
	public BlockSet Border_Top_Right;
	public BlockSet Diagonal_Gate_Left;
	public BlockSet Diagonal_Gate_Right;
	public BlockSet Fork_Bottom_Left;
	public BlockSet Fork_Bottom_Right;
	public BlockSet Fork_Top_Left;
	public BlockSet Fork_Top_Right;
	public BlockSet Funnel_Bottom;
	public BlockSet Funnel_Left;
	public BlockSet Funnel_Right;
	public BlockSet Funnel_Top;
	public BlockSet Taper_Bottom_Left;
	public BlockSet Taper_Bottom_Right;
	public BlockSet Taper_Left_Bottom;
	public BlockSet Taper_Left_Top;
	public BlockSet Taper_Right_Bottom;
	public BlockSet Taper_Right_Top;
	public BlockSet Taper_Top_Left;
	public BlockSet Taper_Top_Right;
	 /*
Border_Bottom_Left;
Border_Bottom_Right;
Border_Top_Left;
Border_Top_Right;
Diagonal_Gate_Left;
Diagonal_Gate_Right;
Fork_Bottom_Left;
Fork_Bottom_Right;
Fork_Top_Left;
Fork_Top_Right;
Funnel_Bottom;
Funnel_Left;
Funnel_Right;
Funnel_Top;
Taper_Bottom_Left;
Taper_Bottom_Right;
Taper_Left_Bottom;
Taper_Left_Top;
Taper_Right_Bottom;
Taper_Right_Top;
Taper_Top_Left;
Taper_Top_Right;
		*/
	
	public void PreRandomiseBlockOrientations(){
		
		BlockSet[] b = GetBlockSetsAsArray();
		
		for(int i = 0; i < b.Length; i++){
			
			if(b[i] == null || b[i].blockSet == null || b[i].blockSet.Length == 0){
				continue;
			}
			
			int index = UnityEngine.Random.Range(0,b[i].blockSet.Length);
			
			b[i].currentBlockIndex = index;
			
		}
		
	}
		
	public BlockSet[] GetBlockSetsAsArray(){
		
		BlockSet[] bs = new BlockSet[54];
		
		bs[0] = Empty;
		bs[1] = Bottom_Left;
		bs[2] = Bottom_Alone;
		bs[3] = Bottom_Right;
		bs[4] = Bottom_Surrounded;
		bs[5] = Right_Alone;
		bs[6] = Centre_Alone;
		bs[7] = Left_Alone;
		bs[8] = Centre_Surrounded;
		bs[9] = Top_Right;
		bs[10] = Top_Alone;
		bs[11] = Top_Left;
		bs[12] = Top_Surrounded;
		bs[13] = Right_Surrounded;
		bs[14] = Left_Surrounded;
		bs[15] = Centre_Horizontal_Surrounded;
		bs[16] = Centre_Vertical_Surrounded;
		bs[17] = Top_Right_Inner_Corner;
		bs[18] = Top_Left_Inner_Corner;
		bs[19] = Bottom_Right_Inner_Corner;
		bs[20] = Bottom_Left_Inner_Corner;
		bs[21] = Right_Diagonal;
		bs[22] = Left_Diagonal;
		bs[23] = Lower_Right_Filled_Diagonal;
		bs[24] = Lower_Left_Filled_Diagonal;
		bs[25] = Upper_Right_Filled_Diagonal;
		bs[26] = Upper_Left_Filled_Diagonal;
		
		bs[27] = Cross_Hub;
		bs[28] = Downward_T_Hub;
		bs[29] = Upward_T_Hub;
		bs[30] = Leftward_T_Hub;
		bs[31] = Rightward_T_Hub;
		
		bs[32] = Border_Bottom_Left;
		bs[33] = Border_Bottom_Right;
		bs[34] = Border_Top_Left;
		bs[35] = Border_Top_Right;
		bs[36] = Diagonal_Gate_Left;
		bs[37] = Diagonal_Gate_Right;
		bs[38] = Fork_Bottom_Left;
		bs[39] = Fork_Bottom_Right;
		bs[40] = Fork_Top_Left;
		bs[41] = Fork_Top_Right;
		bs[42] = Funnel_Bottom;
		bs[43] = Funnel_Left;
		bs[44] = Funnel_Right;
		bs[45] = Funnel_Top;
		bs[46] = Taper_Bottom_Left;
		bs[47] = Taper_Bottom_Right;
		bs[48] = Taper_Left_Bottom;
		bs[49] = Taper_Left_Top;
		bs[50] = Taper_Right_Bottom;
		bs[51] = Taper_Right_Top;
		bs[52] = Taper_Top_Left;
		bs[53] = Taper_Top_Right;
		
		return bs;
	}
	
	public int GetCurrentVariant(){
		
		BlockSet bs = GetCurrentBlockSet();
		
		if(bs != null){
			return bs.currentBlockIndex;
		}
		
		return 0;
	}
	
	public override void ClearDefault ()
	{
		BlockSet[] bs = GetBlockSetsAsArray();
		
		for(int i = 0; i < bs.Length; i++){
			bs[i].SetIsDefault(false);
		}
		
		base.ClearDefault ();
	}
	
	public BlockSet GetDefaultSet(){
		
		//Nothing should be null. This would occur in an instance where we are using the runtime API
		if(Bottom_Left == null){
			return null;
		}
		
		BlockSet[] bs = GetBlockSetsAsArray();
		
		for(int i = 0; i < bs.Length; i++){
			if(bs[i].IsDefault()){
				return bs[i];
			}
		}
		
		
		ClearDefault();
		Centre_Alone.SetIsDefault(true);
		
		return Centre_Alone;
	}
		
	public override GameObject GetDefaultBlock(){
		
		BlockSet bs = GetDefaultSet();
		
		if(bs != null){
			return bs.GetCurrentBlock();
		}
		
		return null;
		
		//PREFAB
		//return this.gameObject;
	}
	
	public GameObject GetCurrentInstantiatedObject(){
		BlockSet bs = GetDefaultSet();
		
		if(bs != null){
			return bs.currentObject;
		}
		
		return null;
	}
	
	public override void ObjectEnteredBlock(TidyMapBoundObject e){
		
		GameObject o = GetCurrentInstantiatedObject();
	
		if(o != null){
			TidyMapBoundObject b = o.GetComponent<TidyMapBoundObject>();
			
			if(b != null){
				
				b.OnObjectEnterBlock(this,e);
				
			}
		}
	
		
	}
	
	public override void ObjectExitedBlock(TidyMapBoundObject e){
		
		GameObject o = GetCurrentInstantiatedObject();
			
		if(o != null){
			TidyMapBoundObject b = o.GetComponent<TidyMapBoundObject>();
			
			if(b != null){
				
				b.OnObjectExitBlock(this,e);
				
			}
		}
	
	}
	
	public override void OnBindToMap (int x, int y, int depth, BlockMap blockMap)
	{
		OnRefreshBlock();
	}
	
	public override void OnShow ()
	{
		BlockSet b = this.GetCurrentBlockSet();
		
		if(b != null){
			b.rotation = new Vector3(x_rotation,y_rotation,z_rotation);
			b.offset = new Vector3(x_offset,y_offset,z_offset);
		}
		
		SetBlockOrientation((BlockOrientation)orientation,false);
		
	}
	
	public override void OnHide ()
	{
		BlockSet b = this.GetCurrentBlockSet();
		if(b != null){
			b.Hide();
		}
	}
	
	public override void OnRefreshBlock ()
	{
		bool[,] blockState = blockMap.GetBlockStateFor(x,y,depth,actAsEmptyBlock);
	
		SetBlockOrientation(TranslateStateToOrientation(blockState),true);
	
	}
	
	public override string ToString ()
	{
		bool[,] blockState =  blockMap.GetBlockStateFor(x,y,depth,actAsEmptyBlock);
		
		return GetStringForState(blockState);
	}
	
	string GetStringForState(bool[,] state){
		
		string s= "";
		
		for(int py = 0; py < 3; py++){
			for(int px = 0; px < 3; px++){
				s += state[px,py] +",";
			}
			s += '\n';
		}
		
		return s;
		
	}
	
	public BlockSet GetSetForOrientation(BlockOrientation orientation, bool includeEmptySets){
		
		BlockSet targetSet = null;
		
		switch(orientation){
			
			case BlockOrientation.Bottom_Alone:{
			
				targetSet = Bottom_Alone;				
			
				break;
			}
			case BlockOrientation.Bottom_Left:{
			
				targetSet = Bottom_Left;	
			
				break;
			}
			case BlockOrientation.Bottom_Right:{
			
				targetSet  = Bottom_Right;
			
				break;
			}
			case BlockOrientation.Bottom_Surrounded:{
			
				targetSet = Bottom_Surrounded;
			
				break;
			}
			case BlockOrientation.Centre_Alone:{
			
				targetSet = Centre_Alone;
			
				break;
			}
			case BlockOrientation.Centre_Horizontal_Surrounded:{
			
				targetSet = Centre_Horizontal_Surrounded;
			
				break;
			}
			case BlockOrientation.Centre_Surrounded:{
			
				targetSet = Centre_Surrounded;
			
				break;
			}
			case BlockOrientation.Centre_Vertical_Surrounded:{
			
				targetSet = Centre_Vertical_Surrounded;
			
				break;
			}
			case BlockOrientation.Left_Alone:{
			
				targetSet = Left_Alone;
			
				break;
			}
			case BlockOrientation.Left_Surrounded:{
			
				targetSet = Left_Surrounded;
			
				break;
			}
			case BlockOrientation.Right_Alone:{
			
				targetSet = Right_Alone;
			
				break;
			}
			case BlockOrientation.Right_Surrounded:{
			
				targetSet = Right_Surrounded;
			
				break;
			}
			case BlockOrientation.Top_Alone:{
			
				targetSet = Top_Alone;
			
				break;
			}
			case BlockOrientation.Top_Left:{
			
				targetSet = Top_Left;
			
				break;
			}
			case BlockOrientation.Top_Right:{
			
				targetSet = Top_Right;
			
				break;
			}
			case BlockOrientation.Top_Surrounded:{
			
				targetSet = Top_Surrounded;
			
				break;
			}
			case BlockOrientation.Bottom_Left_Inner_Corner:{
			
				targetSet = Bottom_Left_Inner_Corner;
			
				break;
			}
			case BlockOrientation.Bottom_Right_Inner_Corner:{
			
				targetSet = Bottom_Right_Inner_Corner;
			
				break;
			}
			case BlockOrientation.Top_Left_Inner_Corner:{
			
				targetSet = Top_Left_Inner_Corner;
			
				break;
			}
			case BlockOrientation.Top_Right_Inner_Corner:{
			
				targetSet = Top_Right_Inner_Corner;
			
				break;
			}
			case BlockOrientation.Left_Diagonal:{
			
				targetSet = Left_Diagonal;
			
				break;
			}
			case BlockOrientation.Right_Diagonal:{
			
				targetSet = Right_Diagonal;
			
				break;
			}
			case BlockOrientation.Lower_Left_Filled_Diagonal:{
			
				targetSet = Lower_Left_Filled_Diagonal;
			
				break;
			}
			case BlockOrientation.Lower_Right_Filled_Diagonal:{
			
				targetSet = Lower_Right_Filled_Diagonal;
			
				break;
			}
			case BlockOrientation.Upper_Left_Filled_Diagonal:{
			
				targetSet = Upper_Left_Filled_Diagonal;
			
				break;
			}
			case BlockOrientation.Upper_Right_Filled_Diagonal:{
			
				targetSet = Upper_Right_Filled_Diagonal;
			
				break;
			}

			case BlockOrientation.Cross_Hub:{
				targetSet = Cross_Hub;
				break;
			}
			case BlockOrientation.Downward_T_Hub:{
				targetSet = Downward_T_Hub;
				break;
			}
			case BlockOrientation.Upward_T_Hub:{
				targetSet = Upward_T_Hub;
				break;
			}
			case BlockOrientation.Leftward_T_Hub:{
				targetSet = Leftward_T_Hub;
				break;
			}
			case BlockOrientation.Rightward_T_Hub:{
				targetSet = Rightward_T_Hub;
				break;
			}
			
			//Gee. Regrets
			//There was probably a better way to do this
			//But here we are
			case BlockOrientation.Border_Bottom_Left:{
			targetSet = Border_Bottom_Left;
			break;
			}
			case BlockOrientation.Border_Bottom_Right:{
			targetSet = Border_Bottom_Right;
			break;
			}
			case BlockOrientation.Border_Top_Left:{
			targetSet = Border_Top_Left;
			break;
			}
			case BlockOrientation.Border_Top_Right:{
			targetSet = Border_Top_Right;
			break;
			}
			case BlockOrientation.Diagonal_Gate_Left:{
			targetSet = Diagonal_Gate_Left;
			break;
			}
			case BlockOrientation.Diagonal_Gate_Right:{
			targetSet = Diagonal_Gate_Right;
			break;
			}
			case BlockOrientation.Fork_Bottom_Left:{
			targetSet = Fork_Bottom_Left;
			break;
			}
			case BlockOrientation.Fork_Bottom_Right:{
			targetSet = Fork_Bottom_Right;
			break;
			}
			case BlockOrientation.Fork_Top_Left:{
			targetSet = Fork_Top_Left;
			break;
			}
			case BlockOrientation.Fork_Top_Right:{
			targetSet = Fork_Top_Right;
			break;
			}
			case BlockOrientation.Funnel_Bottom:{
			targetSet = Funnel_Bottom;
			break;
			}
			case BlockOrientation.Funnel_Left:{
			targetSet = Funnel_Left;
			break;
			}
			case BlockOrientation.Funnel_Right:{
			targetSet = Funnel_Right;
			break;
			}
			case BlockOrientation.Funnel_Top:{
			targetSet = Funnel_Top;
			break;
			}
			case BlockOrientation.Taper_Bottom_Left:{
			targetSet = Taper_Bottom_Left;
			break;
			}
			case BlockOrientation.Taper_Bottom_Right:{
			targetSet = Taper_Bottom_Right;
			break;
			}
			case BlockOrientation.Taper_Left_Bottom:{
			targetSet = Taper_Left_Bottom;
			break;
			}
			case BlockOrientation.Taper_Left_Top:{
			targetSet = Taper_Left_Top;
			break;
			}
			case BlockOrientation.Taper_Right_Bottom:{
			targetSet = Taper_Right_Bottom;
			break;
			}
			case BlockOrientation.Taper_Right_Top:{
			targetSet = Taper_Right_Top;
			break;
			}
			case BlockOrientation.Taper_Top_Left:{
			targetSet = Taper_Top_Left;
			break;
			}
			case BlockOrientation.Taper_Top_Right:{
			targetSet = Taper_Top_Right;
			break;
			}	
		}
		
		if(targetSet == null || !targetSet.HasBlocks() && !includeEmptySets){
			targetSet = GetDefaultSet();
		}
		
		return targetSet;
		
	}
	
	public void Editor_AddToBlockSet(BlockOrientation orientation, GameObject blockObject){
		
		BlockSet targetSet = GetSetForOrientation(orientation,true);
			
		if(targetSet.rootObject == null){
			
			//targetSet.rootObject = new GameObject(orientation.ToString());
			//targetSet.rootObject.transform.parent = transform;
			//targetSet.rootObject.transform.localPosition = Vector3.zero;
			targetSet.rootObject = this.gameObject;
			
		}
		
		
		//PREFAB
		//blockObject.SetActiveRecursively(false);
		
		targetSet.Editor_AddToBlockSet(blockObject, this);
		
	}
	
	public GameObject Editor_RemoveFromBlockSet(BlockOrientation orientation, GameObject blockObject){
		
		BlockSet targetSet = GetSetForOrientation(orientation,true);
		
		if(targetSet == null){
			return null;
		}
		
		GameObject returnObject = targetSet.Editor_RemoveFromBlockSet(blockObject);
		
		if(!targetSet.HasBlocks()){
			
			targetSet.SetIsDefault(false);
			
			BlockSet[] bs = GetBlockSetsAsArray();
			
			for(int i = 0; i < bs.Length; i++){
				
				if(bs[i].HasBlocks()){
					bs[i].SetIsDefault(true);
					break;
				}
				
			}
		}
		
		return returnObject;
		
	}
	
	public void CycleBlockVariation(int direction){
		
		BlockSet bs = GetCurrentBlockSet();
		
		if(bs != null){
						
			bs.CycleIndex(direction);
			
			SetBlockOrientation((BlockOrientation)orientation,true);
		}
		
	}
		
	public BlockSet GetCurrentBlockSet(){
		
		return GetSetForOrientation((BlockOrientation)orientation,false);
		
	}
		
	public void SetBlockOrientation(BlockOrientation orientation, bool refresh){
							
		if(this.orientation == (int)orientation && !refresh){
			return;
		}
		
		Hide();
		
		//Remove this. This is just so i can... look. O_O
		if(orientation != BlockOrientation.Default){
			this.orientation = (int)orientation;
		}
		
		BlockSet chosenSet = GetSetForOrientation(orientation,false);
				
		if(chosenSet == null){
			
			BlockSet d = GetDefaultSet();
			
			if(d != null){
				chosenSet = d;
			}
			else{
				
				//
#if UNITY_4_0
				gameObject.SetActive(true);
#else
				gameObject.active = true;
#endif
				return;
				
			}
			
		}
		
		if(!chosenSet.HasBlocks()){
			
			if(orientation == BlockOrientation.Default){
			}
			else{
				SetBlockOrientation(BlockOrientation.Default,true);
			}
			
			return;
			
		}
				
		chosenSet.Show();
		
#if UNITY_4_0
		gameObject.SetActive(true);
#else
		gameObject.active = true;
#endif
		
		BindChildrenToBlock();
	}
	
	public void SetOffset(Vector3 offset, Vector3 rotation){
		
		x_offset = offset.x;
		y_offset = offset.y;
		z_offset = offset.z;
		
		x_rotation = rotation.x;
		y_rotation = rotation.y;
		z_rotation = rotation.z;
		
		BlockSet[] b = GetBlockSetsAsArray();
		
		for(int i = 0; i < b.Length; i++){
			
			if(b[i] == null){
				continue;
			}
			
			if(b[i].blockSet == null){
				continue;
			}
			
			/*for(int j = 0; j < b[i].blockSet.Length; j++){
				
				if(b[i].blockSet[j] == null){
					continue;
				}
				
				//b[i].blockSet[j].transform.localPosition = offset;
				//b[i].blockSet[j].transform.localRotation = Quaternion.Euler(rotation);
				
				
				
			}*/
			
			b[i].rotation = new Vector3(x_rotation,y_rotation,z_rotation);
			b[i].offset = new Vector3(x_offset,y_offset,z_offset);
			
		}
		
	}
	
	public static BlockOrientation TranslateStateToOrientation(bool[,] blockState){
		
		if(blockState[1,1] == false){
			return BlockOrientation.Empty;
		}
		
		bool upper = blockState[1,0];
		bool left = blockState[0,1];
		bool right = blockState[2,1];
		bool lower = blockState[1,2];
		
		bool upperLeft = blockState[0,0];
		bool upperRight = blockState[2,0];
		bool lowerLeft = blockState[0,2];
		bool lowerRight = blockState[2,2];
		
		//Tapers - very specific
		if(!upper && !upperLeft && !upperRight){
			if(left && right && lower){
				
				if(!lowerLeft){
					return BlockOrientation.Taper_Bottom_Left;	
				}
				
				if(!lowerRight){
					return BlockOrientation.Taper_Bottom_Left;	
				}
				
			}
		}
		
		if(!upperLeft && !left && !lowerLeft){
			if(upper && lower && right){
				
				if(!upperRight){
					return BlockOrientation.Taper_Right_Top;
				}
				
				if(!lowerRight){
					return BlockOrientation.Taper_Right_Bottom;
				}
				
			}
		}
		
		if(!lower && !lowerLeft && !lowerRight){
			if(left && right && upper){
				if(!upperLeft){
					return BlockOrientation.Taper_Top_Left;
				}
				
				if(!upperRight){
					return BlockOrientation.Taper_Top_Right;
				}
			}
		}
		
		if(!right && !lowerRight && !upperRight){
			if(upper && lower && left){
				if(!lowerLeft){
					return BlockOrientation.Taper_Left_Bottom;
				}
				
				if(!upperLeft){
					return BlockOrientation.Taper_Left_Top;
				}
			}
		}
		
		if(!upperLeft && !upperRight && !lowerLeft && !lowerRight){
			
			if(left && right && upper && !lower){
				return BlockOrientation.Upward_T_Hub;	
			}
			
			if(left && right && !upper && lower){
				return BlockOrientation.Downward_T_Hub;
			}
			
			if(left && !right && upper && lower){
				return BlockOrientation.Leftward_T_Hub;
			}
			
			if(!left && right && upper && lower){
				return BlockOrientation.Rightward_T_Hub;
			}
			
			//Borders
			if(right && upper && !left && !lower){
				return BlockOrientation.Border_Bottom_Left;
			}
			
			if(right && !upper && !left && lower){
				return BlockOrientation.Border_Top_Left;
			}
			
			if(!right && upper && left && !lower){
				return BlockOrientation.Border_Bottom_Right;
			}
			
			if(!right && !upper && left && lower){
				return BlockOrientation.Border_Top_Right;
			}
		
		}
		if(upper && left && right && lower){
			
			//Diagonal gates
			if(!upperRight && !lowerLeft && upperLeft && upperRight){
				return BlockOrientation.Diagonal_Gate_Left;
			}
			
			if(upperRight && lowerLeft && !upperLeft && !upperRight){
				return BlockOrientation.Diagonal_Gate_Right;
			}
			
			//Forks
			if(!upperRight && lowerLeft && !upperLeft && !lowerRight){
				return BlockOrientation.Fork_Bottom_Left;
			}
			if(!upperRight && !lowerLeft && !upperLeft && lowerRight){
				return BlockOrientation.Fork_Bottom_Right;
			}
			if(!upperRight && !lowerLeft && upperLeft && !lowerRight){
				return BlockOrientation.Fork_Top_Left;
			}
			if(upperRight && !lowerLeft && !upperLeft && !lowerRight){
				return BlockOrientation.Fork_Top_Right;
			}
			
			//Funnels
			if(!lowerLeft && !lowerRight && upperLeft && upperRight){
				return BlockOrientation.Funnel_Bottom;
			}
			if(!lowerLeft && lowerRight && !upperLeft && upperRight){
				return BlockOrientation.Funnel_Left;
			}
			if(lowerLeft && !lowerRight && upperLeft && !upperRight){
				return BlockOrientation.Funnel_Right;
			}
			if(lowerLeft && lowerRight && !upperLeft && !upperRight){
				return BlockOrientation.Funnel_Top;
			}
			
			if(!upperLeft && !upperRight && !lowerLeft && !lowerRight){
				return BlockOrientation.Cross_Hub;
			}
			
			if(upperLeft && upperRight && lowerLeft && !lowerRight){
				return BlockOrientation.Bottom_Right_Inner_Corner;
			}
			
			if(!upperLeft && upperRight && lowerLeft && lowerRight){
				return BlockOrientation.Top_Left_Inner_Corner;
			}
			
			if(upperLeft && !upperRight && lowerLeft && lowerRight){
				return BlockOrientation.Top_Right_Inner_Corner;
			}
			
			if(upperLeft && upperRight && !lowerLeft && lowerRight){
				return BlockOrientation.Bottom_Left_Inner_Corner;
			}
			
			return BlockOrientation.Centre_Surrounded;
		}
		
		if(!upper && !left && !right && !lower){
			
			if(upperLeft && lowerRight && !upperRight && !lowerLeft){
				return BlockOrientation.Right_Diagonal;
			}
			
			if(!upperLeft && !lowerRight && upperRight && lowerLeft){
				return BlockOrientation.Left_Diagonal;
			}
			
			return BlockOrientation.Centre_Alone;
		}
		
		if(!upper && !left && !upperLeft && right && lower && upperRight && lowerLeft){
			
			return BlockOrientation.Upper_Left_Filled_Diagonal;
			
		}
		
		if(!upper && !right && !upperRight && left && lower && upperLeft && lowerRight){
			
			return BlockOrientation.Upper_Right_Filled_Diagonal;
			
		}
		
		if(!lower && right && lowerRight && !left && !lowerLeft && upperLeft && upper){
			
			return BlockOrientation.Lower_Left_Filled_Diagonal;
			
		}
		
		if(!lower && !right && !lowerRight && left && lowerLeft && upperRight && upper){
			
			return BlockOrientation.Lower_Right_Filled_Diagonal;
			
		}
				
		if(left && right && upper){
			return BlockOrientation.Bottom_Surrounded;
		}
		
		if(left && right && lower){
			return BlockOrientation.Top_Surrounded;
		}
		
		if(upper && lower && left){
			return BlockOrientation.Right_Surrounded;
		}
		
		if(upper && lower && right){
			return BlockOrientation.Left_Surrounded;
		}
		
		if(left && right){
			return BlockOrientation.Centre_Horizontal_Surrounded;
		}
		
		if(upper && lower){
			return BlockOrientation.Centre_Vertical_Surrounded;
		}
		
		if(upper && left){
			return BlockOrientation.Bottom_Right;
		}
		
		if(upper && right){
			return BlockOrientation.Bottom_Left;
		}
		
		if(lower && left){
			return BlockOrientation.Top_Right;
		}
		
		if(lower && right){
			return BlockOrientation.Top_Left;
		}
		
		if(right){
			return BlockOrientation.Left_Alone;
		}
		
		if(left){
			return BlockOrientation.Right_Alone;
		}
		
		if(lower){
			return BlockOrientation.Top_Alone;
		}
		
		if(upper){
			return BlockOrientation.Bottom_Alone;
		}
		
		
		return BlockOrientation.Default;
	}
	
	public override void RandomiseVariant ()
	{
		BlockSet b = GetCurrentBlockSet();
		
		if(b == null){
			return;
		}
		
		b.RandomiseVariant();
		
		BindChildrenToBlock();
	}
	
	public override void SetVariant (int variant)
	{
		BlockSet b = GetCurrentBlockSet();
		
		if(b == null){
			return;
		}
		
		b.SetIndex(variant);
		
		BindChildrenToBlock();
	}

}


