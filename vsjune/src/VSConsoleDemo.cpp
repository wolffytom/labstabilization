#include "stdafx.h"
#include "VSConsoleDemo.h"


VSConsoleDemo::VSConsoleDemo()
{
	std::cout << "VS-June ConsoleDemo 0.0---12th_June_2016" << std::endl;

	VSStabHandle *stabHandle = new VSStabHandle();
	stabHandle->OpenAVideo("input.avi");

	system("pause");
}


VSConsoleDemo::~VSConsoleDemo()
{
}
