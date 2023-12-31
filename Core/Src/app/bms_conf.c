

UINT8 SeriesNum = 16;

// 不同串数维护的表格
const unsigned char SeriesSelect_AFE1[16][16] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 1串
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 2串
	{0, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 3   76920
	{0, 1, 2, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 4   76920
	{0, 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 5   76920
	//{0 ,1 ,2 ,3 ,4 ,15,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0, 0},   	//6   76920 + AD	//第6串映射到16串，刘总说的果然有用
	{0, 1, 4, 5, 6, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 6   76930 		//刘总说的没用了，改为930，外扩怕了
	{0, 1, 2, 4, 5, 6, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 7   76930
	{0, 1, 2, 4, 5, 6, 7, 9, 0, 0, 0, 0, 0, 0, 0, 0},	   // 8   76930
	{0, 1, 2, 3, 4, 5, 6, 7, 9, 0, 0, 0, 0, 0, 0, 0},	   // 9   76930
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0},	   // 10  76930
	{0, 1, 3, 4, 5, 6, 7, 9, 10, 11, 14, 0, 0, 0, 0, 0},   // 11  76940
	{0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 14, 0, 0, 0, 0},  // 12  76940
	{0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 14, 0, 0, 0},  // 13  76940
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 0, 0},  // 14  76940
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0}, // 15  76940
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15} // 16  76940 + AD	//这个版本不会有16串了
};