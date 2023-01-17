#include <fstream>
#include <string>

class Store{
public:
	Store();
	~Store();
	void open();
	void open(std::string p);
	void close();
	void write(std::string t);
	std::string path;
	std::ofstream writeFile;
};
