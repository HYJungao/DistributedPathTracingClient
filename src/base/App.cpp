#define _CRT_SECURE_NO_WARNINGS

#include "App.hpp"
#include "base/Main.hpp"
#include "gpu/GLContext.hpp"
#include "3d/Mesh.hpp"
#include "io/File.hpp"
#include "io/StateDump.hpp"
#include "base/Random.hpp"

#include "RayTracer.hpp"
#include "rtlib.hpp"

#include <stdio.h>
#include <conio.h>
#include <map>
#include <algorithm>

#include <iostream>
#include <fstream>
#include <string>

using namespace FW;


//------------------------------------------------------------------------

struct control {
	bool m_RTMode;
	bool m_JBF;
	bool m_normalMapped;
	bool m_useRussianRoulette;
	int m_kernel;
	int m_spp_server;
	int m_numBounces;
};

struct LightControl {
	Vec3f position;
	Mat3f orientation;
};

bool fileExists(std::string fileName)
{
	return std::ifstream(fileName).good();
}

App::App(std::vector<std::string>& cmd_args)
	: m_commonCtrl(CommonControls::Feature_Default & ~CommonControls::Feature_RepaintOnF5),
	m_cameraCtrl(&m_commonCtrl, CameraControls::Feature_Default | CameraControls::Feature_StereoControls),
	m_action(Action_None),
	m_cullMode(CullMode_None),
	m_numBounces(1),
	m_lightSize(0.25f),
	m_RTMode(false),
	m_useRussianRoulette(false),
	m_normalMapped(false),
	m_img(Vec2i(10, 10), ImageFormat::RGBA_Vec4f) // will get resized immediately
{
	m_routerContext = zmq::context_t(1);
	m_router = zmq::socket_t(m_routerContext, zmq::socket_type::router);
	m_router.bind("tcp://*:5556");

	m_frameContext = zmq::context_t(1);
	m_frameRouter = zmq::socket_t(m_frameContext, zmq::socket_type::router);
	m_frameRouter.bind("tcp://*:5557");

	m_commonCtrl.showFPS(true);
	m_commonCtrl.addStateObject(this);
	m_cameraCtrl.setKeepAligned(true);

	m_JBF = false;
	m_JBF_server = false;
	m_kernel = 6;
	m_spp = 4;
	m_spp_server = 4;
	m_commonCtrl.addToggle(&m_JBF, FW_KEY_NONE, "Enable Joint Bilateral Filtering(slow)");
	m_commonCtrl.addToggle(&m_JBF_server, FW_KEY_NONE, "Enable Joint Bilateral Filtering(slow) on server");
	m_commonCtrl.beginSliderStack();
	m_commonCtrl.addSlider(&m_kernel, 1, 64, false, FW_KEY_NONE, FW_KEY_NONE, "Kernel Size of Joint Bilateral Filtering= %d", 0, &clear_on_next_frame);
	m_commonCtrl.addSlider(&m_spp, 1, 512, false, FW_KEY_NONE, FW_KEY_NONE, "Sample Per Pixel= %d", 0, &clear_on_next_frame);
	m_commonCtrl.addSlider(&m_spp_server, 1, 512, false, FW_KEY_NONE, FW_KEY_NONE, "Sample Per Pixel of Server= %d", 0, &clear_on_next_frame);
	m_commonCtrl.addSlider(&m_numBounces, 0, 8, false, FW_KEY_NONE, FW_KEY_NONE, "Number of indirect bounces= %d", 0, &clear_on_next_frame);
	m_commonCtrl.endSliderStack();

	//m_commonCtrl.addButton((S32*)&m_action, Action_LoadMesh, FW_KEY_M, "Load mesh or state... (M)");
	//m_commonCtrl.addButton((S32*)&m_action, Action_ReloadMesh, FW_KEY_F5, "Reload mesh (F5)");
	//m_commonCtrl.addButton((S32*)&m_action, Action_SaveMesh, FW_KEY_O, "Save mesh... (O)");
	//m_commonCtrl.addButton((S32*)&m_action, Action_LoadBVH, FW_KEY_NONE, "Load BVH from file...");
	//m_commonCtrl.addSeparator();

	//m_commonCtrl.addButton((S32*)&m_action, Action_ResetCamera, FW_KEY_NONE, "Reset camera");
	//m_commonCtrl.addButton((S32*)&m_action, Action_EncodeCameraSignature, FW_KEY_NONE, "Encode camera signature");
	//m_commonCtrl.addButton((S32*)&m_action, Action_DecodeCameraSignature, FW_KEY_NONE, "Decode camera signature...");
	m_window.addListener(&m_cameraCtrl);
	//m_commonCtrl.addSeparator();

	//m_commonCtrl.addButton((S32*)&m_action, Action_NormalizeScale, FW_KEY_NONE, "Normalize scale");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FlipXY,                  FW_KEY_NONE,    "Flip X/Y");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FlipYZ,                  FW_KEY_NONE,    "Flip Y/Z");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FlipZ,                   FW_KEY_NONE,    "Flip Z");
	//m_commonCtrl.addSeparator();

	//m_commonCtrl.addButton((S32*)&m_action, Action_NormalizeNormals, FW_KEY_NONE, "Normalize normals");
	//m_commonCtrl.addButton((S32*)&m_action, Action_FlipNormals, FW_KEY_NONE, "Flip normals");
	//m_commonCtrl.addButton((S32*)&m_action, Action_RecomputeNormals, FW_KEY_NONE, "Recompute normals");
	//m_commonCtrl.addSeparator();

	//m_commonCtrl.addToggle((S32*)&m_cullMode, CullMode_None, FW_KEY_NONE, "Disable backface culling");
	//m_commonCtrl.addToggle((S32*)&m_cullMode, CullMode_CW, FW_KEY_NONE, "Cull clockwise faces");
	//m_commonCtrl.addToggle((S32*)&m_cullMode, CullMode_CCW, FW_KEY_NONE, "Cull counter-clockwise faces");
	//m_commonCtrl.addButton((S32*)&m_action, Action_FlipTriangles, FW_KEY_NONE, "Flip triangles");
	//m_commonCtrl.addSeparator();

	//    m_commonCtrl.addButton((S32*)&m_action, Action_CleanMesh,               FW_KEY_NONE,    "Remove unused materials, denegerate triangles, and unreferenced vertices");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_CollapseVertices,        FW_KEY_NONE,    "Collapse duplicate vertices");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_DupVertsPerSubmesh,      FW_KEY_NONE,    "Duplicate vertices shared between multiple materials");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FixMaterialColors,       FW_KEY_NONE,    "Override material colors with average over texels");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_DownscaleTextures,       FW_KEY_NONE,    "Downscale textures by 2x");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_ChopBehindNear,          FW_KEY_NONE,    "Chop triangles behind near plane");
	//    m_commonCtrl.addSeparator();

	m_commonCtrl.addButton((S32*)&m_action, Action_PathTraceMode, FW_KEY_ENTER, "Path trace mode (ENTER)");
	m_commonCtrl.addButton((S32*)&m_action, Action_PlaceLightSourceAtCamera, FW_KEY_SPACE, "Place light at camera (SPACE)", &clear_on_next_frame);
	//m_commonCtrl.addButton(&m_clearVisualization, FW_KEY_BACKSPACE, "Clear visualization (BACKSPACE)");
	m_commonCtrl.addToggle(&m_useRussianRoulette, FW_KEY_NONE, "Use Russian Roulette", &clear_on_next_frame);
	m_commonCtrl.addToggle(&m_normalMapped, FW_KEY_NONE, "Use normal mapping", &clear_on_next_frame);
	//m_commonCtrl.addToggle(&m_playbackVisualization, FW_KEY_NONE, "Visualization playback");
	//m_commonCtrl.beginSliderStack();
	//m_commonCtrl.addSlider(&m_numBounces, 0, 8, false, FW_KEY_NONE, FW_KEY_NONE, "Number of indirect bounces= %d", 0, &clear_on_next_frame);
	//m_commonCtrl.addSlider(&m_lightSize, 0.01f, 200.0f, false, FW_KEY_NONE, FW_KEY_NONE, "Light source area= %f", 0, &clear_on_next_frame);
	//m_commonCtrl.endSliderStack();
	//m_commonCtrl.beginSliderStack();
	//m_commonCtrl.addSlider(&m_numDebugPathCount, 1, 1000, false, FW_KEY_NONE, FW_KEY_NONE, "Number of debug paths to fire= %d");
	//m_commonCtrl.addSlider(&m_visualizationAlpha, 0.01f, 1.0f, false, FW_KEY_NONE, FW_KEY_NONE, "Debug ray visualization alpha= %f");
	//m_commonCtrl.endSliderStack();

	m_window.addListener(this);
	m_window.addListener(&m_commonCtrl);

	m_window.setTitle("Client");
	m_commonCtrl.setStateFilePrefix("state_assignment3_");

	m_window.setSize(Vec2i(800, 600));
	m_pathtrace_renderer.reset(new PathTraceRenderer);
	m_areaLight.reset(new AreaLight);

	process_args(cmd_args);

	m_commonCtrl.loadState(m_commonCtrl.getStateFileName(1));

	m_timer.start();
	m_lastTimestamp = std::chrono::steady_clock::now();
}

// returns the index of the needle in the haystack or -1 if not found
int find_argument(std::string needle, std::vector<std::string> haystack) {

	for (unsigned j = 0; j < haystack.size(); ++j)
		if (!haystack[j].compare(needle))
			return j;
	
	return -1;
}

void App::process_args(std::vector<std::string>& args) {

	// all of the possible cmd arguments and the corresponding enums (enum value is the index of the string in the vector)
	const std::vector<std::string> argument_names = { "-builder", "-spp", "-output_images", "-use_textures", "-bat_render", "-aa", "-ao", "-ao_length" };
	enum argument { arg_not_found = -1, builder = 0, spp = 1, output_images = 2, use_textures = 3, bat_render = 4, AA = 5, AO = 6, AO_length = 7 };

	// similarly a list of the implemented BVH builder types
	const std::vector<std::string> builder_names = { "none", "sah", "object_median", "spatial_median", "linear" };
	enum builder_type { builder_not_found = -1, builder_None = 0, builder_SAH = 1, builder_ObjectMedian = 2, builder_SpatialMedian = 3, builder_Linear = 4 };

	m_settings.batch_render = false;
	m_settings.output_images = false;
	m_settings.use_textures = true;
	m_settings.sample_type = AA_sampling;
	m_settings.ao_length = 1.0f;
	m_settings.spp = 1;
	m_settings.splitMode = SplitMode_Sah;

	for (unsigned i = 0; i < args.size(); ++i) {

		// try to recognize the argument
		argument cmd = argument(find_argument(args[i], argument_names));

		switch (cmd) {

		case bat_render:
			m_settings.batch_render = true;
			break;

		case output_images:
			m_settings.output_images = true;
			break;
		
		case use_textures:
			m_settings.use_textures = true;
			break;
		
		case AO:
			m_settings.sample_type = AO_sampling;
			break;

		case AA:
			m_settings.sample_type = AA_sampling;
			break;

		case spp:
			++i;
			m_settings.spp = std::stoi(args[i]);
			break;

		case AO_length:
			++i;
			m_settings.ao_length = std::stof(args[i]);
			break;

		case builder: {

			++i;
			builder_type type = builder_type(find_argument(args[i], builder_names));

			if (type==builder_not_found) {
				type = builder_SAH;
				std::cout << "BVH builder not recognized, using Surface Area Heuristic" << std::endl;
				break;
			}

			switch (type) {

			case builder_None:
				m_settings.splitMode = SplitMode_None;
				break;
			
			case builder_SAH:
				m_settings.splitMode = SplitMode_Sah;
				break;

			case builder_ObjectMedian:
				m_settings.splitMode = SplitMode_ObjectMedian;
				break;

			case builder_SpatialMedian:
				m_settings.splitMode = SplitMode_SpatialMedian;
				break;

			case builder_Linear:
				m_settings.splitMode = SplitMode_Linear;
				break;
			}

			break;
		}


		default:
			if (args[i][0] == '-')std::cout << "argument \"" << args[i] << "\" not found!" << std::endl;

		}
	}
	if (m_settings.batch_render)
		m_settings.use_textures = false;
}

//------------------------------------------------------------------------

App::~App()
{
}

//------------------------------------------------------------------------

bool App::handleEvent(const Window::Event& ev)
{
	if (ev.type == Window::EventType_Close)
	{
		m_window.showModalMessage("Exiting...");
		bool temp[2] = { true, true };
		m_cameraCtrl.sendControl(&temp, 2 * sizeof(bool));
		delete this;
		return true;
	}

	Action action = m_action;
	m_action = Action_None;
	String name;
	Mat4f mat;

	switch (action)
	{
	case Action_None:
		break;


	case Action_LoadMesh:
		name = m_window.showFileLoadDialog("Load mesh or state", getMeshImportFilter()+",dat:State file");
		if (name.getLength())
			if (name.endsWith(".dat"))
				m_commonCtrl.loadState(name);
			else
				loadMesh(name);
		break;

	case Action_ReloadMesh:
		if (m_meshFileName.getLength())
			loadMesh(m_meshFileName);
		break;

	case Action_SaveMesh:
		name = m_window.showFileSaveDialog("Save mesh", getMeshExportFilter());
		if (name.getLength())
			saveMesh(name);
		break;

	case Action_LoadBVH:
		name = m_window.showFileLoadDialog("Load bvh", "hierarchy:BVH");
		m_rt->loadHierarchy(name.getPtr(), m_rtTriangles);
		break;

	case Action_ResetCamera:
		if (m_mesh)
		{
			m_cameraCtrl.initForMesh(m_mesh.get());
			m_commonCtrl.message("Camera reset");
		}
		break;

	case Action_EncodeCameraSignature:
		m_window.setVisible(false);
		printf("\nCamera signature:\n");
		printf("%s\n", m_cameraCtrl.encodeSignature().getPtr());
		waitKey();
		break;

	case Action_DecodeCameraSignature:
	{
		m_window.setVisible(false);
		printf("\nEnter camera signature:\n");

		char buf[1024];
		if (scanf_s("%s", buf, FW_ARRAY_SIZE(buf)))
			m_cameraCtrl.decodeSignature(buf);
		else
			setError("Signature too long!");

		if (!hasError())
			printf("Done.\n\n");
		else
		{
			printf("Error: %s\n", getError().getPtr());
			clearError();
			waitKey();
		}
	}
		break;

	case Action_NormalizeScale:
		if (m_mesh)
		{
			Vec3f lo, hi;
			m_mesh->getBBox(lo, hi);
			m_mesh->xform(Mat4f::scale(Vec3f(2.0f / (hi - lo).max())) * Mat4f::translate((lo + hi) * -0.5f));
		}
		break;

	case Action_FlipXY:
		std::swap(mat.col(0), mat.col(1));
		if (m_mesh)
		{
			m_mesh->xform(mat);
			m_mesh->flipTriangles();
		}
		break;

	case Action_FlipYZ:
		std::swap(mat.col(1), mat.col(2));
		if (m_mesh)
		{
			m_mesh->xform(mat);
			m_mesh->flipTriangles();
		}
		break;

	case Action_FlipZ:
		mat.col(2) = -mat.col(2);
		if (m_mesh)
		{
			m_mesh->xform(mat);
			m_mesh->flipTriangles();
		}
		break;

	case Action_NormalizeNormals:
		if (m_mesh)
			m_mesh->xformNormals(mat.getXYZ(), true);
		break;

	case Action_FlipNormals:
		mat = -mat;
		if (m_mesh)
			m_mesh->xformNormals(mat.getXYZ(), false);
		break;

	case Action_RecomputeNormals:
		if (m_mesh)
			m_mesh->recomputeNormals();
		break;

	case Action_FlipTriangles:
		if (m_mesh)
			m_mesh->flipTriangles();
		break;

	case Action_CleanMesh:
		if (m_mesh)
			m_mesh->clean();
		break;

	case Action_CollapseVertices:
		if (m_mesh)
			m_mesh->collapseVertices();
		break;

	case Action_DupVertsPerSubmesh:
		if (m_mesh)
			m_mesh->dupVertsPerSubmesh();
		break;

	case Action_FixMaterialColors:
		if (m_mesh)
			m_mesh->fixMaterialColors();
		break;

	case Action_DownscaleTextures:
		if (m_mesh)
			downscaleTextures(m_mesh.get());
		break;

	case Action_ChopBehindNear:
		if (m_mesh)
		{
			Mat4f worldToClip = m_cameraCtrl.getCameraToClip() * m_cameraCtrl.getWorldToCamera();
			Vec4f pleq = worldToClip.getRow(2) + worldToClip.getRow(3);
			chopBehindPlane(m_mesh.get(), pleq);
		}
		break;
	case Action_PlaceLightSourceAtCamera: {
		m_areaLight->setOrientation(m_cameraCtrl.getCameraToWorld().getXYZ());
		m_areaLight->setPosition(m_cameraCtrl.getPosition());
		initState.m_lightOrientation = m_areaLight->getOrientation();
		initState.m_lightPosition = m_areaLight->getPosition();
		LightControl temp = { m_areaLight->getPosition(),m_areaLight->getOrientation() };
		m_cameraCtrl.sendControl(&temp, sizeof(LightControl));
		m_commonCtrl.message("Placed light at camera");
		break;
	}

	case Action_PathTraceMode:
		m_RTMode = !m_RTMode;
		if (m_RTMode)
		{
			control ctl = { m_RTMode, m_JBF_server, m_normalMapped, m_useRussianRoulette, m_kernel, m_spp_server, m_numBounces };
			m_cameraCtrl.sendControl(&ctl, sizeof(control));
			initState.m_RTMode = m_RTMode;
			initState.m_JBF_server = m_JBF_server;
			initState.m_normalMapped = m_normalMapped;
			initState.m_useRussianRoulette = m_useRussianRoulette;
			initState.m_kernel = m_kernel;
			initState.m_spp_server = m_spp_server;
			initState.m_numBounces = m_numBounces;
			m_pathtrace_renderer->stop();
			if (m_img.getSize() != m_window.getSize())
			{
				// Replace m_img with a new Image. TODO: Clean this up.
				Vec2f tmp = m_window.getSize();
				m_cameraCtrl.sendControl(&tmp, sizeof(Vec2f));
				m_img.~Image();
				new (&m_img) Image(m_window.getSize(), ImageFormat::RGBA_Vec4f);	// placement new, will get autodestructed
			}
			m_pathtrace_renderer->setNormalMapped(m_normalMapped);
			m_pathtrace_renderer->setJBF(m_JBF);
			m_pathtrace_renderer->setKernel(m_kernel);
			m_pathtrace_renderer->setSPP(m_spp);
			m_pathtrace_renderer->startPathTracingProcess(m_mesh.get(), m_areaLight.get(), m_rt.get(), &m_img, m_useRussianRoulette ? -m_numBounces : m_numBounces, m_cameraCtrl);
		}
		else
		{
			initState.m_RTMode = m_RTMode;
			m_cameraCtrl.sendControl(&m_RTMode, sizeof(bool));
			m_pathtrace_renderer->stop();
		}
		break;
	default:
		FW_ASSERT(false);
		break;
		}

	if (ev.type == Window::EventType_KeyUp)
	{
		if (ev.key == FW_KEY_CONTROL)
		{
			Vec2i pos = ev.mousePos;
			m_pathtrace_renderer->stop();
			m_visualization.clear();
			m_pathtrace_renderer->debugVis = true;

			Random rnd;
			Vec3f nn;
			Vec3f position;
			for (int i = 0; i < m_numDebugPathCount; ++i)
				m_pathtrace_renderer->tracePath(pos.x, pos.y, m_pathtrace_renderer->m_context, -1, rnd, m_visualization, nn, position, Mat4f(0));

			m_pathtrace_renderer->debugVis = false;
			m_RTMode = false;
		}
	}


	auto currentTimestamp = std::chrono::steady_clock::now();
	if (std::chrono::duration_cast<std::chrono::seconds>(currentTimestamp - m_lastTimestamp).count() >= 6) {
		if (m_servers.size() != m_aliveServers.size()) {
			std::cout << "some servers is dead" << std::endl;

			m_servers = std::move(m_aliveServers);

			int blockId = 0;
			int blockNum = m_servers.size();
			for (auto& it : m_servers) {
				zmq::message_t message(sizeof(int) * 2);
				std::memcpy(static_cast<int*>(message.data()), &blockId, sizeof(int));
				std::memcpy(static_cast<int*>(message.data()) + 1, &blockNum, sizeof(int));

				m_router.send(zmq::buffer(it.first), zmq::send_flags::sndmore);
				m_router.send(message, zmq::send_flags::none);

				it.second = blockId;
				blockId++;
			}
		}
		else {
			for (auto& it : m_servers) {
				// send heart heat
				m_router.send(zmq::buffer(it.first), zmq::send_flags::sndmore);
				m_router.send(zmq::buffer("1"), zmq::send_flags::none);
			}
		}

		m_lastTimestamp = std::chrono::steady_clock::now();
		m_aliveServers.clear();
	}

	// send client state to initialize server or
	// re-schedule load when some servers disconnect
	zmq::message_t identity;
	bool received = m_router.recv(identity, zmq::recv_flags::dontwait).has_value();
	if (received) {
		std::string clientID(static_cast<char*>(identity.data()), identity.size());

		zmq::message_t request;
		m_router.recv(request, zmq::recv_flags::none);
		std::string requestData(static_cast<char*>(request.data()), request.size());
		// std::cout << "Received from " << clientID << ": " << requestData << std::endl;

		if (m_servers.find(clientID) != m_servers.end()) {
			m_aliveServers.emplace(clientID, m_servers[clientID]);
			// std::cout << "receive heart beat from" << clientID << std::endl;
		}
		else {
			int blockNum = m_servers.size() + 1;

			// send current client state
			initState.m_position = m_cameraCtrl.getPosition();
			initState.m_forward = m_cameraCtrl.getForward();
			initState.m_up = m_cameraCtrl.getUp();
			initState.m_size = m_window.getSize();
			initState.m_fov = m_cameraCtrl.getFOV();
			initState.m_blockId = m_servers.size();
			initState.m_blockNum = blockNum;
			zmq::message_t message(sizeof(InitialState) + m_scene.getLength() + 1);
			std::memcpy(message.data(), &initState, sizeof(InitialState));
			std::memcpy(static_cast<char*>(message.data()) + sizeof(InitialState), m_scene.getPtr(), m_scene.getLength() + 1);
			m_router.send(identity, zmq::send_flags::sndmore);
			m_router.send(message, zmq::send_flags::none);

			m_cameraCtrl.sendControl(&blockNum, sizeof(int));

			m_servers.emplace(clientID, m_servers.size());
			m_aliveServers.emplace(clientID, m_servers[clientID]);
		}

		//// send current client state
		//initState.m_position = m_cameraCtrl.getPosition();
		//initState.m_forward = m_cameraCtrl.getForward();
		//initState.m_up = m_cameraCtrl.getUp();
		//initState.m_fov = m_cameraCtrl.getFOV();
		//zmq::message_t message(sizeof(InitialState) + m_scene.getLength() + 1);
		//std::memcpy(message.data(), &initState, sizeof(InitialState));
		//std::memcpy(static_cast<char*>(message.data()) + sizeof(InitialState), m_scene.getPtr(), m_scene.getLength() + 1);
		//m_router.send(identity, zmq::send_flags::sndmore);
		//m_router.send(message, zmq::send_flags::none);
	}

	m_window.setVisible(true);

	if (ev.type == Window::EventType_Paint)
		renderFrame(m_window.getGL());
	m_window.repaint();
	return false;
		}

//------------------------------------------------------------------------

void App::readState(StateDump& d)
{
	String meshFileName;

	d.pushOwner("App");
	d.get(meshFileName, "m_meshFileName");
	d.get((S32&)m_cullMode, "m_cullMode");
	d.get((S32&)m_numBounces, "m_numBounces");
	d.get((bool&)m_useRussianRoulette, "m_useRussianRoulette");
	d.popOwner();

	m_areaLight->readState(d);
	m_lightSize = m_areaLight->getSize().x;	// dirty; doesn't allow for rectangular lights, only square. TODO

	if (m_meshFileName != meshFileName && meshFileName.getLength()) {
		m_cameraCtrl.sendControl(m_commonCtrl.m_currentScene.getPtr(), m_commonCtrl.m_currentScene.getLength() + 1);
		m_scene = m_commonCtrl.m_currentScene;
		initState.m_lightPosition = m_areaLight->getPosition();
		initState.m_lightOrientation = m_areaLight->getOrientation();
		loadMesh(meshFileName);
	}
	}

//------------------------------------------------------------------------

void App::writeState(StateDump& d) const
{
	d.pushOwner("App");
	d.set(m_meshFileName, "m_meshFileName");
	d.set((S32)m_cullMode, "m_cullMode");
	d.set((S32&)m_numBounces, "m_numBounces");
	d.set((bool&)m_useRussianRoulette, "m_useRussianRoulette");
	d.popOwner();

	m_areaLight->writeState(d);
}

//------------------------------------------------------------------------

void App::waitKey(void)
{
	printf("Press any key to continue . . . ");
	_getch();
	printf("\n\n");
}

//------------------------------------------------------------------------

void App::renderFrame(GLContext* gl)
{
	// Setup transformations.

	Mat4f worldToCamera = m_cameraCtrl.getWorldToCamera();
	Mat4f projection = gl->xformFitToView(Vec2f(-1.0f, -1.0f), Vec2f(2.0f, 2.0f)) * m_cameraCtrl.getCameraToClip();
	Mat4f worldToClip = projection * worldToCamera;

	if (worldToClip != previous_camera || clear_on_next_frame)
	{
		previous_camera = worldToClip;
		clear_on_next_frame = false;

		m_pathtrace_renderer->stop();

		if (m_img.getSize() != m_window.getSize())
		{
			// Replace m_img with a new Image. TODO: Clean this up.
			Vec2f tmp = m_window.getSize();
			m_cameraCtrl.sendControl(&tmp, sizeof(Vec2f));
			m_img.~Image();
			new (&m_img) Image(m_window.getSize(), ImageFormat::RGBA_Vec4f);	// placement new, will get autodestructed
		}

		m_pathtrace_renderer->startPathTracingProcess(m_mesh.get(), m_areaLight.get(), m_rt.get(), &m_img, m_useRussianRoulette ? -m_numBounces : m_numBounces, m_cameraCtrl);
	}

	glClearColor(0.2f, 0.4f, 0.8f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_RTMode)
	{
		// if we are computing radiosity, refresh mesh colors every 0.5 seconds
		if (m_pathtrace_renderer->isRunning())
		{
			m_pathtrace_renderer->updatePicture(&m_img);
			m_pathtrace_renderer->checkFinish();

			// restart cycle
			m_updateClock.start();
		}
		else if (m_pathtrace_renderer->m_notDenoised)
		{
			m_pathtrace_renderer->m_notDenoised = false;
			m_pathtrace_renderer->denoise(&m_img);
		}
		else 
		{
			
			zmq::message_t identity;
			bool received = m_frameRouter.recv(identity, zmq::recv_flags::dontwait).has_value();
			if (received) {
				std::string clientID(static_cast<char*>(identity.data()), identity.size());

				zmq::message_t frame;
				m_frameRouter.recv(frame, zmq::recv_flags::none);

				// std::cout << "Received frame" << std::endl;

				int blockId = m_servers[clientID];
				int blockNum = m_servers.size();

				int image_height = m_img.getSize().y;
				int vBlockHeight = image_height / blockNum;
				int vBlockStart = blockId * vBlockHeight;
				int vHeight = blockId == blockNum - 1 ? image_height - vBlockStart : vBlockHeight;

				m_pathtrace_renderer->pixelColor.resize(vHeight * m_img.getSize().x);
				memcpy(m_pathtrace_renderer->pixelColor.data(), frame.data(), frame.size());
				m_pathtrace_renderer->blendFrame(&m_img, vBlockStart, vHeight);
			}
		}

		gl->drawImage(m_img, Vec2f(0));

		return;
	}

	// Initialize GL state.

	glEnable(GL_DEPTH_TEST);

	if (m_cullMode == CullMode_None)
		glDisable(GL_CULL_FACE);
	else
	{
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glFrontFace((m_cullMode == CullMode_CW) ? GL_CCW : GL_CW);
	}

	// No mesh => skip.

	if (!m_mesh)
	{
		gl->drawModalMessage("No mesh loaded!");
		return;
	}

	// Render.

	if (!gl->getConfig().isStereo)
		renderScene(gl, worldToCamera, projection);
	else
	{
		glDrawBuffer(GL_BACK_LEFT);
		renderScene(gl, m_cameraCtrl.getCameraToLeftEye() * worldToCamera, projection);
		glDrawBuffer(GL_BACK_RIGHT);
		glClear(GL_DEPTH_BUFFER_BIT);
		renderScene(gl, m_cameraCtrl.getCameraToRightEye() * worldToCamera, projection);
		glDrawBuffer(GL_BACK);
	}

	m_areaLight->setSize(Vec2f(m_lightSize));
	m_areaLight->draw(worldToCamera, projection);

	if (m_clearVisualization)
		m_visualization.clear();
	m_clearVisualization = false;
	if (m_visualization.size() > 0)
	{

		glPushAttrib(GL_ALL_ATTRIB_BITS);

		glDepthMask(false); // Disables depth write but not read

		glMatrixMode(GL_MODELVIEW); // Set the projection and camera matrices
		glLoadMatrixf(worldToCamera.getPtr());
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(projection.getPtr());

		glEnable(GL_BLEND);
		glBlendEquation(GL_FUNC_ADD);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE);
		float alpha = m_visualizationAlpha;

		int m_visualizationStepSize = 100;
		for (int i = 0; i < m_visualization.size(); ++i)
		{
			auto& node = m_visualization[i];
			if (m_playbackVisualization && (m_currentVisualizationIndex < i || m_currentVisualizationIndex > i + m_visualizationStepSize)) // Only render one node if playback enabled
				continue;

			glBegin(GL_LINES);

			for (auto& line : node.lines)
			{
				glColor4f(line.color.x, line.color.y, line.color.z, alpha);
				glVertex3f(line.start.x, line.start.y, line.start.z);
				glVertex3f(line.stop.x, line.stop.y, line.stop.z);
			}

			glEnd();
		}
		m_currentVisualizationIndex = (m_currentVisualizationIndex + 1) % m_visualization.size();

		if (!m_playbackVisualization)
			for (auto& node : m_visualization)
				for (auto& label : node.labels)
				{
					Vec4f posNDC = worldToClip * Vec4f(label.position, 1.0f);
					if (posNDC.z < .0f)
						continue;
					Vec2f pixelpos = posNDC.getXY() / posNDC.w;
					gl->drawLabel(label.text.c_str(), pixelpos, 0xffffffff);
				}

		glPopAttrib();
	}
	// Display status line.

	m_commonCtrl.message(sprintf("Triangles = %d, vertices = %d, materials = %d",
		m_mesh->numTriangles(),
		m_mesh->numVertices(),
		m_mesh->numSubmeshes()),
		"meshStats");
}

//------------------------------------------------------------------------

void App::renderScene(GLContext* gl, const Mat4f& worldToCamera, const Mat4f& projection)
{
	// Draw mesh.
	if (m_mesh) {
		m_mesh->draw(gl, worldToCamera, projection);
	}
}


//------------------------------------------------------------------------




const static F32 texAttrib[] =
{
	0, 1,
	1, 1,
	0, 0,
	1, 0
};





//------------------------------------------------------------------------


void App::loadMesh(const String& fileName)
{
	m_clearVisualization = true;
	m_pathtrace_renderer->stop();
	m_RTMode = false;

	// find the scene name; the file path without preceding folders and file extension

	std::string path = std::string(fileName.getPtr());

	size_t begin = path.find_last_of("/\\")+1,
		     end = path.find_last_of(".");

	m_results.scene_name = path.substr(begin, end - begin);

	std::cout << "Scene name: " << m_results.scene_name << std::endl;

	m_window.showModalMessage(sprintf("Loading mesh from '%s'...", fileName.getPtr()));
	String oldError = clearError();
	std::unique_ptr<MeshBase> mesh((MeshBase*)importMesh(fileName));

	String newError = getError();

	if (restoreError(oldError))
	{
		m_commonCtrl.message(sprintf("Error while loading '%s': %s", fileName.getPtr(), newError.getPtr()));
		return;
	}

	m_meshFileName = fileName;

	// convert input mesh to colored format
	m_mesh.reset(new MeshWithColors(*mesh));
	


	// fix input colors to white so we see something
	for ( S32 i = 0; i < m_mesh->numVertices(); ++i )
		m_mesh->mutableVertex(i).c = Vec3f(1, 1, 1);

	m_commonCtrl.message(sprintf("Loaded mesh from '%s'", fileName.getPtr()));

	// build the BVH!
	constructTracer();
}

//------------------------------------------------------------------------

void App::saveMesh(const String& fileName)
{
	if (!m_mesh)
	{
		m_commonCtrl.message("No mesh to save!");
		return;
	}

	m_window.showModalMessage(sprintf("Saving mesh to '%s'...", fileName.getPtr()));
	String oldError = clearError();
	exportMesh(fileName, m_mesh.get());
	String newError = getError();

	if (restoreError(oldError))
	{
		m_commonCtrl.message(sprintf("Error while saving '%s': %s", fileName.getPtr(), newError.getPtr()));
		return;
	}

	m_meshFileName = fileName;
	m_commonCtrl.message(sprintf("Saved mesh to '%s'", fileName.getPtr()));
}

//------------------------------------------------------------------------

// This function iterates over all the "sub-meshes" (parts of the object with different materials),
// heaps all the vertices and triangles together, and calls the BVH constructor.
// It is the responsibility of the tree to free the data when deleted.
// This functionality is _not_ part of the RayTracer class in order to keep it separate
// from the specifics of the Mesh class.
void App::constructTracer()
{
	// fetch vertex and triangle data ----->
	m_rtTriangles.clear();
	m_rtTriangles.reserve(m_mesh->numTriangles());


	for (int i = 0; i < m_mesh->numSubmeshes(); ++i)
	{
		const Array<Vec3i>& idx = m_mesh->indices(i);
		for (int j = 0; j < idx.getSize(); ++j)
		{

			const VertexPNTC &v0 = m_mesh->vertex(idx[j][0]),
						     &v1 = m_mesh->vertex(idx[j][1]),
							 &v2 = m_mesh->vertex(idx[j][2]);

			RTTriangle t = RTTriangle(v0, v1, v2);

			t.m_data.vertex_indices = idx[j];
			t.m_material = &(m_mesh->material(i));

			m_rtTriangles.push_back(t);
		}
	}


	// compute checksum

	m_rtVertexPositions.clear();
	m_rtVertexPositions.reserve(m_mesh->numVertices());
	for (int i = 0; i < m_mesh->numVertices(); ++i)
		m_rtVertexPositions.push_back(m_mesh->vertex(i).p);

	String md5 = RayTracer::computeMD5(m_rtVertexPositions);
	FW::printf("Mesh MD5: %s\n", md5.getPtr());

	// construct a new ray tracer (deletes the old one if there was one)
	m_rt.reset(new RayTracer());

	// whether we want to try loading a saved hierarchy from disk
	bool tryLoadHierarchy = false;

	// always construct when measuring performance
	if (m_settings.batch_render)
		tryLoadHierarchy = false;

	if (tryLoadHierarchy)
	{
		// check if saved hierarchy exists

		std::string meshName = m_meshFileName.getPtr();
		std::string hierarchyName = meshName.substr(0, meshName.find_last_of("."));
#ifdef _WIN64
		hierarchyName += "_x64";
#endif
		
		hierarchyName += ".hierarchy";

		String hierarchyCacheFile = hierarchyName.c_str();

		if (fileExists(hierarchyCacheFile.getPtr()))
		{
			// yes, load!
			m_rt->loadHierarchy(hierarchyCacheFile.getPtr(), m_rtTriangles);
			::printf("Loaded hierarchy from %s\n", hierarchyCacheFile.getPtr());
		}
		else
		{
			// no, construct...
			LARGE_INTEGER start, stop, frequency;
			QueryPerformanceFrequency(&frequency);
			QueryPerformanceCounter(&start); // Start time stamp		

			m_rt->constructHierarchy(m_rtTriangles, m_settings.splitMode);

			QueryPerformanceCounter(&stop); // Stop time stamp

			int build_time = (int)((stop.QuadPart - start.QuadPart) * 1000.0 / frequency.QuadPart); // Get timer result in milliseconds
			std::cout << "Build time: " << build_time << " ms" << std::endl;
			// .. and save!
			m_rt->saveHierarchy(hierarchyCacheFile.getPtr(), m_rtTriangles);
			::printf("Saved hierarchy to %s\n", hierarchyCacheFile.getPtr());
		}
	}
	else
	{
		// nope, bite the bullet and construct it

		LARGE_INTEGER start, stop, frequency;
		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&start); // Start time stamp		
		
		m_rt->constructHierarchy(m_rtTriangles, m_settings.splitMode);

		QueryPerformanceCounter(&stop); // Stop time stamp

		m_results.build_time = (int)((stop.QuadPart - start.QuadPart) * 1000.0 / frequency.QuadPart); // Get timer result in milliseconds
		std::cout << "Build time: " << m_results.build_time << " ms"<< std::endl;
	}

}



//------------------------------------------------------------------------

void App::downscaleTextures(MeshBase* mesh)
{
	FW_ASSERT(mesh);
	Hash<const Image*, Texture> hash;
	for (int submeshIdx = 0; submeshIdx < mesh->numSubmeshes(); submeshIdx++)
		for (int textureIdx = 0; textureIdx < MeshBase::TextureType_Max; textureIdx++)
		{
		Texture& tex = mesh->material(submeshIdx).textures[textureIdx];
		if (tex.exists())
		{
			const Image* orig = tex.getImage();
			if (!hash.contains(orig))
			{
				Image* scaled = orig->downscale2x();
				hash.add(orig, (scaled) ? Texture(scaled, tex.getID()) : tex);
			}
			tex = hash.get(orig);
		}
		}
}

//------------------------------------------------------------------------

void App::chopBehindPlane(MeshBase* mesh, const Vec4f& pleq)
{
	FW_ASSERT(mesh);
	int posAttrib = mesh->findAttrib(MeshBase::AttribType_Position);
	if (posAttrib == -1)
		return;

	for (int submeshIdx = 0; submeshIdx < mesh->numSubmeshes(); submeshIdx++)
	{
		Array<Vec3i>& inds = mesh->mutableIndices(submeshIdx);
		int triOut = 0;
		for (int triIn = 0; triIn < inds.getSize(); triIn++)
		{
			if (dot(mesh->getVertexAttrib(inds[triIn].x, posAttrib), pleq) >= 0.0f ||
				dot(mesh->getVertexAttrib(inds[triIn].y, posAttrib), pleq) >= 0.0f ||
				dot(mesh->getVertexAttrib(inds[triIn].z, posAttrib), pleq) >= 0.0f)
			{
				inds[triOut++] = inds[triIn];
			}
		}
		inds.resize(triOut);
	}

	mesh->clean();
}


//------------------------------------------------------------------------

void FW::init(std::vector<std::string>& args)
{
	new App(args);
}

//------------------------------------------------------------------------

bool App::fileExists(const String& fn)
{
	FILE* pF = fopen(fn.getPtr(), "rb");
	if (pF != 0)
	{
		fclose(pF);
		return true;
	}
	else
	{
		return false;
	}
}

//------------------------------------------------------------------------
