#pragma once


#include "gui/Window.hpp"
#include "gui/CommonControls.hpp"
#include "gui/Image.hpp"
#include "3d/CameraControls.hpp"
#include "gpu/Buffer.hpp"

#include <vector>
#include <memory>
#include <map>
#include <set>
#include <chrono>

#include "RayTracer.hpp"

#include "AreaLight.hpp"
#include "PathTraceRenderer.hpp"

#include <zmq.hpp>


namespace FW {

struct InitialState
{
    Vec3f m_position;
    Vec3f m_forward;
    Vec3f m_up;
    Vec3f m_lightPosition;
    Mat3f m_lightOrientation;
    Vec2f m_size;
    F32 m_fov;
    bool m_RTMode = false;
    bool m_JBF_server = false;
    bool m_normalMapped = false;
    bool m_useRussianRoulette = false;
    int m_kernel = 6;
    int m_spp_server = 4;
    int m_numBounces = 1;
    int m_blockId = 0;
    int m_blockNum = 0;
};

//------------------------------------------------------------------------

class App : public Window::Listener, public CommonControls::StateObject
{
private:
    enum Action
    {
        Action_None,

        Action_LoadMesh,
        Action_ReloadMesh,
        Action_SaveMesh,
		Action_LoadBVH,

        Action_ResetCamera,
        Action_EncodeCameraSignature,
        Action_DecodeCameraSignature,

        Action_NormalizeScale,
        Action_FlipXY,
        Action_FlipYZ,
        Action_FlipZ,

        Action_NormalizeNormals,
        Action_FlipNormals,
        Action_RecomputeNormals,

        Action_FlipTriangles,

        Action_CleanMesh,
        Action_CollapseVertices,
        Action_DupVertsPerSubmesh,
        Action_FixMaterialColors,
        Action_DownscaleTextures,
		Action_ChopBehindNear,

        Action_PathTraceMode,
        Action_PlaceLightSourceAtCamera
    };

    enum CullMode
    {
        CullMode_None = 0,
        CullMode_CW,
        CullMode_CCW,
    };

    struct RayVertex
    {
        Vec3f       pos;
        U32         color;
    };

	enum bvh_build_method { None, SAH };
	enum SamplingType { AO_sampling, AA_sampling };
	// this structure holds the necessary arguments when rendering using command line parameters
	struct {
		bool batch_render;
		SplitMode splitMode;		// the BVH builder to use
		int spp;					// samples per pixel to use
		SamplingType sample_type;	// AO or AA sampling; AO includes one extra sample for the primary ray
		bool output_images;			// might be useful to compare images with the example
		bool use_textures;			// whether or not textures are used
		bool use_arealights;		// whether or not area light sampling is used
		bool enable_reflections;	// whether to compute reflections in whitted integrator
		float ao_length;			
	} m_settings;
	
	struct {
		std::string state_name;										// filenames of the state and scene files
		std::string scene_name;
		int rayCount;
		int build_time, trace_time;

	} m_results;

public:
					 App			(std::vector<std::string>& cmd_args);
    virtual         ~App            (void);

    virtual bool    handleEvent     (const Window::Event& ev);
    virtual void    readState       (StateDump& d);
    virtual void    writeState      (StateDump& d) const;

private:
	void			process_args	(std::vector<std::string>& args);

    void            waitKey         (void);
    void            renderFrame     (GLContext* gl);
    void            renderScene     (GLContext* gl, const Mat4f& worldToCamera, const Mat4f& projection);
    void            loadMesh        (const String& fileName);
    void            saveMesh        (const String& fileName);
    void            loadRayDump     (const String& fileName);

    static void     downscaleTextures(MeshBase* mesh);
    static void     chopBehindPlane (MeshBase* mesh, const Vec4f& pleq);

    static bool		fileExists		(const String& fileName);

    // 
	void			constructTracer(void);

	void			blitRttToScreen(GLContext* gl);

    void            scheduleBlockPartition();


private:
                    App             (const App&); // forbidden
    App&            operator=       (const App&); // forbidden

private:
    Window          m_window;
    CommonControls  m_commonCtrl;
    CameraControls  m_cameraCtrl;

    Action          m_action;
    String          m_meshFileName;
    CullMode        m_cullMode;
	Timer			m_timer;

    std::unique_ptr<RayTracer>			m_rt;
	std::vector<Vec3f>				    m_rtVertexPositions; // kept only for MD5 checksums
    std::vector<RTTriangle>				m_rtTriangles;

    std::unique_ptr<MeshWithColors>     m_mesh;
	std::unique_ptr<AreaLight>          m_areaLight;
    std::unique_ptr<PathTraceRenderer>	m_pathtrace_renderer;
	int									m_numBounces;
	float								m_lightSize;
	Timer								m_updateClock;

	bool								m_RTMode;
	bool								m_useRussianRoulette;
	bool								m_normalMapped;

	bool								clear_on_next_frame = false;
	Mat4f								previous_camera = Mat4f(0);
	Image								m_img;
	int									m_numDebugPathCount = 1;
	int									m_currentVisualizationIndex = 0;
	float								m_visualizationAlpha = 1;
	bool								m_playbackVisualization = false;
	bool								m_clearVisualization = false;

	std::vector<PathVisualizationNode> m_visualization;

    bool                                m_JBF;
    bool                                m_JBF_server;
    int                                 m_kernel;
    int                                 m_spp;
    int                                 m_spp_server;

public:
    zmq::context_t m_context;
    zmq::socket_t m_socket;

    zmq::context_t m_routerContext;
    zmq::socket_t m_router;

    String m_scene;
    InitialState initState;

    std::map<std::string, int> m_servers;
    std::map<std::string, int> m_aliveServers;

    std::chrono::steady_clock::time_point m_lastTimestamp;
};


//------------------------------------------------------------------------
}
