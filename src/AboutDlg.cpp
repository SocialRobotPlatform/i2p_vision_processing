

#include "AboutDlg.h"
#include <vector>
#include <cstdlib>
#include <fstream>
#include <string>
#include <iostream>
#include "afxdialogex.h"
#include "ctype.h"

using namespace std;

HWND AbouthWndApp;

AboutDlg::AboutDlg()
{

}


AboutDlg::~AboutDlg()
{
}


INT_PTR CALLBACK DialogProc1(HWND, UINT, WPARAM, LPARAM);
int AboutDlg::Run(HINSTANCE hInstance, int nCmdShow)
{
	MSG       msg = { 0 };
	WNDCLASS  wc;

	// Dialog custom window class
	ZeroMemory(&wc, sizeof(wc));
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hInstance = hInstance;
	wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP5));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"aboutDlg";


	if (!RegisterClassW(&wc))
	{
		return 0;
	}

	// Create main application window
	AbouthWndApp = CreateDialogParamW(
		hInstance,
		MAKEINTRESOURCE(IDD_ABOUTDIALOG),
		NULL,
		DialogProc1,
		0);

	// Show window
	ShowWindow(AbouthWndApp, nCmdShow);

	// Main message loop
	while (WM_QUIT != msg.message)
	{
		if (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
		{
			// If it's a dialog message it will be taken care of by the dialog proc
			if ((AbouthWndApp != NULL) && IsDialogMessageW(AbouthWndApp, &msg))
			{
				continue;
			}

			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
	}

	return static_cast<int>(msg.wParam);
}


INT_PTR CALLBACK DialogProc1(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{

	int wmId, wmEvent;
	PAINTSTRUCT ps;
	HDC hdc;
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		wmId = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDOK:
			break;
		}
		break;
	case WM_PAINT:
		hdc = BeginPaint(hDlg, &ps);
		// TODO: Add any drawing code here...
		EndPaint(hDlg, &ps);
		break;
	case WM_DESTROY:
		//PostQuitMessage(0);
		ShowWindow(AbouthWndApp, SW_SHOW);
		break;
	case WM_CLOSE:
		//PostQuitMessage(0);
		ShowWindow(AbouthWndApp, SW_SHOW);
		break;
	case WM_TIMER:

		break;
	}
	return (INT_PTR)FALSE;
}

