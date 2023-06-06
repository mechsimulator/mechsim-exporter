#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <functional>
#include <utility>
#include <sstream>
#include <filesystem>

#include "ThreadPool.h"
#include "Parser.h"
#include "Common.h"
#include "Math.h"

const std::string WORKSPACE_ID			 = "FusionSolidEnvironment";
const std::string PANEL_ID				 = "MechSim_Panel";
const std::string CMD_ID				 = "MechSim_Export";
const std::string CMD_NAME				 = "MechSim Export";
const std::string CMD_DESCRIPTION		 = "Export to MechSim.";
const std::string COMMAND_BESIDE_ID		 = "ScriptsManagerCommand";
const std::string MECHSIM_DIR			 = "C:\\Users\\Public\\MechSim";
const std::string MECHSIM_ASSEMBLIES_DIR = std::string(MECHSIM_DIR).append("\\assemblies");

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

#define ms_format(x) ((std::ostringstream() << x).str())

Ptr<Application> app;
Ptr<UserInterface> ui;

std::ofstream logfile;
std::ofstream file;

struct MSExporterSettings {
	std::string path;
} settings;

class OnInputChangedEventHander : public adsk::core::InputChangedEventHandler
{
public:
	void notify(const Ptr<InputChangedEventArgs>& eventArgs) override {
		app->log("input changed");
	}
};

class OnExecuteEventHander : public CommandEventHandler
{
public:
	void notify(const Ptr<CommandEventArgs>& eventArgs) override {
		
		Ptr<Design> design = app->activeProduct();
		auto root = design->rootComponent();

		std::string path = std::string(settings.path).append(root->name().append(".mrr").insert(0, "\\"));
		app->log(path);
		file.open(path, std::ios::binary);

		Parser parser(root);
		parser.parse();
		parser.serialize(file);
	}
};

class OnDestroyEventHandler : public CommandEventHandler
{
public:
	void notify(const Ptr<CommandEventArgs>& eventArgs) override {
		adsk::terminate();
	}
};

class CommandCreatedEventHandler : public adsk::core::CommandCreatedEventHandler 
{
public:
	void notify(const Ptr<CommandCreatedEventArgs>& eventArgs) override {
		if (eventArgs) {
			Ptr<Command> cmd = eventArgs->command();
			if (cmd) {
				app->log("command created");
				Ptr<CommandInputs> inputs = cmd->commandInputs();
				
				Ptr<StringValueCommandInput> pathInput = inputs->addStringValueInput("MechSim_ExportFilePath", "Export Path", MECHSIM_ASSEMBLIES_DIR);

				settings.path = pathInput->value();

				Ptr<CommandEvent> onExec = cmd->execute();
				onExec->add(&onExecuteHandler);
			}
		}
	}

private:
	OnExecuteEventHander onExecuteHandler;
	OnDestroyEventHandler onDestroyHandler;
	OnInputChangedEventHander onInputChangedHandler;
} _commandCreatedEventHandler;

static void verify_directory(const std::string& path) {
	if (!std::filesystem::is_directory(path))
		std::filesystem::create_directory(path);
}

extern "C" XI_EXPORT bool run(const char* context)
{
	app = Application::get();
	if (!app) {
		return false;
	}

	ui = app->userInterface();
	if (!ui) {
		return false;
	}

	verify_directory(MECHSIM_DIR);
	verify_directory(MECHSIM_ASSEMBLIES_DIR);

	Ptr<CommandDefinition> cmdDef = ui->commandDefinitions()->addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESCRIPTION, "./Resources");
	
	cmdDef->commandCreated()->add(&_commandCreatedEventHandler);

	auto workspace = ui->workspaces()->itemById(WORKSPACE_ID);
	auto panel = workspace->toolbarPanels()->add(PANEL_ID, "MECHSIM");
	auto control = panel->controls()->addCommand(cmdDef, COMMAND_BESIDE_ID, false);
	
	control->isPromoted(true);
	
	cmdDef->execute();
	
	adsk::autoTerminate();

	return true;
}

extern "C" XI_EXPORT bool stop(const char* context)
{
	auto workspace = ui->workspaces()->itemById(WORKSPACE_ID);
	auto panel = workspace->toolbarPanels()->itemById(PANEL_ID);
	auto command_control = panel->controls()->itemById(CMD_ID);
	auto command_definition = ui->commandDefinitions()->itemById(CMD_ID);

	if (command_control) {			
		command_control->deleteMe();
	}

	if (command_definition) {
		command_definition->deleteMe();
	}

	panel->deleteMe();

	logfile.close();
	file.close();

	if (ui) {
		app->log("addin stopped");
		ui = nullptr;
	}

	return true;
}


#ifdef XI_WIN

#include <windows.h>

BOOL APIENTRY DllMain(HMODULE hmodule, DWORD reason, LPVOID reserved)
{
	switch (reason)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#endif // XI_WIN
