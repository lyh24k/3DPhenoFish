#include "SQL.h"

FishSql::FishSql() {
	res = NULL;
}

bool FishSql::ConnectDatabase(const char hostname[], const char username[], const char password[], const char databasename[])
{
	mysql_init(&mysql);
	if (!(mysql_real_connect(&mysql, hostname, username, password, databasename, 0, NULL, 0)))
	{
		printf("Error connecting to database:%s\n", mysql_error(&mysql));
		return false;

	}
	else
	{
		printf("Connected...\n");
		return true;
	}

}

void FishSql::FreeConnect()
{
	mysql_free_result(res);
	mysql_close(&mysql);
	mysql_server_end();
	cout << "Mysql closed..." << endl;
}

bool FishSql::QueryData(const char query[])
{
	if (mysql_query(&mysql, query))
	{
		printf("Query failed (%s)\n", mysql_error(&mysql));
		return false;
	}
	if (!(res = mysql_store_result(&mysql)))
	{
		printf("Couldn't get result from %s\n", mysql_error(&mysql));
		return false;

	}
	//打印数据行数
	//printf("number of dataline returned: %d\n", mysql_affected_rows(&mysql));

	//获取字段的信息
	//char *str_field[32];  //定义一个字符串数组存储字段信息
	for (int i = 0; fd = mysql_fetch_field(res); i++)   //在已知字段数量的情况下获取字段名
	{
		strcpy(field[i], fd->name);
	}
	int n = mysql_num_fields(res);
	for (int i = 0; i<n; i++)   //打印字段
		printf("%10s\t", field[i]);
	printf("\n");
	//打印获取的数据
	while (column = mysql_fetch_row(res))   //在已知字段数量情况下，获取并打印下一行
	{
		for (int i = 0; i < n; i++)
			printf("%10s\t", column[i]);
		printf("\n");
	}
	return true;
}

bool FishSql::InsertData(const char insert[])
{
	if (mysql_query(&mysql, insert))        //执行SQL语句
	{
		printf("Query failed (%s)\n", mysql_error(&mysql));
		return false;
	}
	else
	{
		printf("Insert success\n");
		return true;
	}

}

bool FishSql::ModifyData(const char modify[])
{
	if (mysql_query(&mysql, modify))        //执行SQL语句
	{
		printf("Query failed (%s)\n", mysql_error(&mysql));
		return false;
	}
	else
	{
		printf("Insert success\n");
		return true;
	}

}

bool FishSql::DeleteData(const char del[])
{
	if (mysql_query(&mysql, del))        //执行SQL语句
	{
		printf("Query failed (%s)\n", mysql_error(&mysql));
		return false;
	}
	else
	{
		printf("Insert success\n");
		return true;
	}

}