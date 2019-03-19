#include <iostream>
#include <math.h>
#include <set> 
#include <iterator> 

#define ROW 324
#define COL 324

struct cell{
	int parent_x;
	int parent_y;
	double f, g, h;
}

bool isInMap(int row, int col){
	return (row>=0) && (row < ROW) && (col>=0) && (col < COL);
}

bool isUnBlocked(int grid[][COL], int row, int col){
	if(grid[row][col] == 1){
		return true;
	}
	else{
		return false;
	}
}


bool isDestination(int row, int col, std::pair<int,int> destination){
	if(row==destination.first && col==destination.second){
		return true;
	}
	else{
		return false;
	}
}

double euclidDist(int x1, int y1, int x2, int y2){
	return ((double) sqrt( (pow((x2-x1),2)) + pow((y2-y1),2) ) );
}

void findPath(int grid[][COL], std::pair<int,int> start, std::pair<int,int> dest){
	if(!isInMap(start.first, start.second)){
		return;
	}
	
	if(!isInMap(dest.first, dest.second)){
		return;
	}
	
	if((!isUnBlocked(grid, start.first, start.second)) || (!isUnBlocked(grid, dest.first, dest.second)) ){
		return;
	}
	
	if(isDestination(start.frist,start.second,dest)){
		return;
	}
		
	//CLOSED LIST
	bool closedList[ROW][COL];
	memset(closedList, false, sizeof (closedList));
	
	cell cellDetails[ROW][COL];

	int x,y;
	
	for(x=0;x<ROW;x++){
		for(y=0;y<COL;y++){
			cellDetails[x][y].f = 10000;
			cellDetails[x][y].g = 10000;
			cellDetails[x][y].h = 10000;
			cellDetails[x][y].parent_x = -1;
			cellDetails[x][y].parent_y = -1;
		}
	}	
	
	x = start.first;
	y = start.second;
	cellDetails[x][y].f = 0;
	cellDetails[x][y].g = 0;
	cellDetails[x][y].h = 0;
	cellDetails[x][y].parent_x = x;
	cellDetails[x][y].parent_y = y;
	
	std::set< std::pair<double, std::pair<int, int> > > openList;
	
	openList.insert(std::make_pair(0.0, std::make_pair(x,y)));
	
	bool atDest = false;

	while(!openList.empty()){
		std::pair<double, std::pair<int, int> > p = *openList.begin();
	        openList.erase(openList.begin()); 
		
		x = p.second.first;
		y = p.second.second;
		closedList[i][j] = true;
		double gNew, hNew, fNew;
		/*Cell-->Popped Cell (i, j) 
	        N -->  North       (i-1, j) 
		S -->  South       (i+1, j) 
		E -->  East        (i, j+1) 
		W -->  West           (i, j-1) 
		N.E--> North-East  (i-1, j+1) 
		N.W--> North-West  (i-1, j-1) 
		S.E--> South-East  (i+1, j+1) 
		S.W--> South-West  (i+1, j-1)*/
				
		if(isInMap(x-1,y)){
			if(isDestination(x-1,y,dest)){
				cellDetails[x-1][y].parent_x = x;
				cellDetails[x-1][y].parent_y = y;
				std::cout<< "Found Destination";
				foundDest = true;
				return;
			}
			else if(closedList[x-1][y]==false && isUnBlocked(grid, x-1,y)==true){
				gNew = cellDetails[x][y].g+1.0;
				hNew = euclidDist(x-1,y,dest.first,dest.second);
				fNew = gNew + hNew;
				if(cellDetails[x-1][y].f = FLT_MAX || cellDetails)				

			}
		}
	}	
}
