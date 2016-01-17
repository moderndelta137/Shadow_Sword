using System;
using System.Collections.Generic;
using System.Text;

namespace DopplerInteractive.TidyTileMapper.Utilities.Pathing
{
	public interface IPathMap
	{
	    int GetMapWidth();
	    int GetMapHeight();
	    bool IsWalkable(int x, int y, int z, bool pathOverAllBlocks);
	}
}

