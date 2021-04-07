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
	MYSQL mysql;//mysql����
	MYSQL_FIELD *fd;//�ֶ�������
	char field[32][32];//���ֶ�����ά����
	MYSQL_RES *res;//�����е�һ����ѯ�����
	MYSQL_ROW column;//��ʾ�����е���
					 //char query[150];//��ѯ���
public:
	FishSql();
	bool ConnectDatabase(const char hostname[], const char username[], const char password[], const char databasename[]);
	void FreeConnect();
	bool QueryData(const char query[]);
	bool InsertData(const char insert[]);
	bool ModifyData(const char modify[]);
	bool DeleteData(const char del[]);
};