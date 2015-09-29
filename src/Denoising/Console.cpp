#include <Windows.h>
#include <io.h>
#include <fcntl.h>
#include <iomanip>
#include "Console.h"

CConsoleOutput* CConsoleOutput::m_instance = NULL;
CConsoleOutput::CConsoleOutput()
{
	//creat a new console to the process.
	AllocConsole();
	int hCrun;       
	hCrun = _open_osfhandle((long)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);   
	file  = _fdopen(hCrun, "w");   
	// use default stream buffer   
	setvbuf(file, NULL, _IONBF, 0);   
	*stdout = *file;   

	std::cout << "\tReady! Debug Informations...\n\n";   
}

CConsoleOutput::~CConsoleOutput()
{
	FreeConsole();
	fclose(file);
}

void CConsoleOutput::Destory()
{
	if (m_instance)
	{
		delete m_instance;
		m_instance = NULL;
	}
	return ;
}

CConsoleOutput* CConsoleOutput::Instance()
{
	if (m_instance==NULL)
	{
		m_instance = new CConsoleOutput;
	}
	return m_instance;
}
