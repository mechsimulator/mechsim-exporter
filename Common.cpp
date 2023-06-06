#include "Common.h"

void error(const char* title, const char* msg) {
	ui->messageBox(msg, title);
	adsk::terminate();
}

void warn(const char* title, const char* msg) {
	ui->messageBox(msg, title);
}