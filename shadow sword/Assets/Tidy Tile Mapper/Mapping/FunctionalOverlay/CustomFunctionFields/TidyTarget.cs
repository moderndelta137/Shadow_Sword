using System;
using UnityEngine;

namespace DopplerInteractive.TidyTileMapper.FunctionalOverlay
{
	[Serializable]
	public class TidyTarget
	{
		public Block TidyTarget_parentBlock;
		public Vector3 targetDifference;
		
		public TidyTarget (Block parentBlock, Block targetBlock)
		{
			this.TidyTarget_parentBlock = parentBlock;
			this.targetDifference = new Vector3(
			                                    	parentBlock.x - targetBlock.x,
			                                    	parentBlock.y - targetBlock.y,
			                                    	parentBlock.depth - targetBlock.depth			                                    
			                                    );
		}
		
		public Vector3 GetTargetCoordinates(){
			if(TidyTarget_parentBlock == null){
				Debug.LogWarning("Parent Block is null for Tidy Target");
				return Vector3.zero;
			}
			
			return new Vector3(
			                   	TidyTarget_parentBlock.x - targetDifference.x,
			                   	TidyTarget_parentBlock.y - targetDifference.y,
			                   	TidyTarget_parentBlock.depth - targetDifference.z
			                   );
		}
	}
}

