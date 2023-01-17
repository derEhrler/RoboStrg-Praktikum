/*
 * Store.cpp
 *
 *  Created on: 21.10.2016
 *      Author: yitao
 */
#include "Store.h"

Store::Store(){

}
Store::~Store(){
	close();
}

void Store::open(){
	writeFile.open("data5.txt");
}
void Store::open(std::string p){
	path = p;
	writeFile.open(path);
}
void Store::close(){
	writeFile.close();
}
void Store::write(std::string t){
	writeFile << t << std::endl;
}
