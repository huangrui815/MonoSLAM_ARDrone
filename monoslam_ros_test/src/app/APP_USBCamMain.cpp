#include "APP_MoSLAMThread.h"
#include "app/SL_GlobParam.h"
#include "app/SL_MoSLAM.h"
#include "app/APP_MyApp.h"
#include "app/APP_SynObj.h"

#include "tools/SL_Tictoc.h"
#include "tools/SL_Timing.h"
#include "slam/SL_SLAMHelper.h"

bool usbCamMain() {
	/////////////////////////1.GPU initilization/////////////////////////
	//initialization for CG;
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(" ");
	glutHideWindow();
	GLenum err = glewInit();
	if (GLEW_OK != err)
		cout << "ERROR: could not initialize GLEW!" << endl;
	
	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "OpenGL vendor: " << glGetString(GL_VENDOR) << endl;
	cout << "OpenGL renderer: " << glGetString(GL_RENDERER) << endl << endl;

	V3D_GPU::Cg_ProgramBase::initializeCg();

	MoSLAM* moSLAM = MyApp::moSLAM;

	try {
		while (!MyApp::bExit) {

			//initialization
			moSLAM->init(false);

				//notify the GUI thread to create GUIs
				broadcastCreateGUI();
				//wait for the accomplishment of creating GUIs
				waitCreateGUI();

			//update the data associated with the GUIs
			MyApp::videoWnd->setSLAMData(moSLAM);

			if (MyApp::modelWnd1)
				MyApp::modelWnd1->setSLAMData(moSLAM);
			if (MyApp::modelWnd2)
				MyApp::modelWnd2->setSLAMData(moSLAM);

			//start online SLAM process	
			int i = 0;
			while (!MyApp::bExit) {
				TimeMeasurer tm;
				tm.tic();
				moSLAM->grabReadFrame();
				moSLAM->processOneFrame();
				if( MyApp::bStartInit && !moSLAM->mpMaker.doInit)
					moSLAM->startMapInit();
				if( !MyApp::bStartInit && moSLAM->mpMaker.doInit)
					moSLAM->endMapInit();
				
				moSLAM->m_tmPerStep = tm.toc();
				updateDisplayData();
				i++;
			}
			cout << " the result is saved at " << MyApp::timeStr << endl;
			moSLAM->exportResults(MyApp::timeStr);
		}
	} catch (SL_Exception& e) {
		logInfo(e.what());
#ifdef WIN32
		wxMessageBox(e.what());
#endif
	} catch (std::exception& e) {
		logInfo("%s\n", e.what());
		logInfo("slam failed!\n");
#ifdef WIN32
		wxMessageBox(e.what());
#endif
	}
	MyApp::bRunning = false;
	logInfo("slam exits\n");
	return true;
}
