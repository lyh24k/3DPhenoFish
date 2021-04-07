#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <mysql.h>
#include <winsock.h>

using namespace std;

#pragma comment(lib,"wsock32.lib")

class FishSql {

private:
	MYSQL mysql;//mysql连接
	MYSQL_FIELD *fd;//字段列数组
	char field[32][32];//存字段名二维数组
	MYSQL_RES *res;//返回行的一个查询结果集
	MYSQL_ROW column;//表示数据行的列
					 //char query[150];//查询语句
public:
	FishSql();
	bool ConnectDatabase(const char hostname[], const char username[], const char password[], const char databasename[]);
	void FreeConnect();
	bool QueryData(const char query[]);
	bool InsertData(const char insert[]);
	bool ModifyData(const char modify[]);
	bool DeleteData(const char del[]);
};