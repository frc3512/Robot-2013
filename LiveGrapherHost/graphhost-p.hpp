#ifndef _GRAPHHOST_CPP_H
#define _GRAPHHOST_CPP_H

#include <string>

class GraphHost {
public:
	GraphHost(int port);
	~GraphHost();
	int graphData(float x, float y, std::string dataset);
	
private:
	struct graphhost_t *inst;
	
};

#endif
