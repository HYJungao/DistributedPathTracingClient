Designed a distributed rendering system using ZeroMQ: client-side ray tracing for direct lighting and server-side path tracing for indirect lighting, with results integrated on the client, enabling collaborative rendering across devices in LAN.

Developed a real-time synchronization mechanism where the client broadcasts user inputs (e.g., camera controls, scene updates, parameter adjustments), and servers subscribe to ensure consistent states.

Supported automatic new server detection, initial state synchronization on connection, and pixel block reallocation and distribution for dynamic load balancing.

Enabled fallback to purely local rendering, ensuring basic interaction even in unstable network environments, which is suitable for game streaming and AR/VR in LAN, maximizing idle device utilization to deliver high-quality application.