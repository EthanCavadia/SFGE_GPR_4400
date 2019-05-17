/*
 MIT License

 Copyright (c) 2017 SAE Institute Switzerland AG

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */
#ifndef SFGE_JSON_UTILITY_H
#define SFGE_JSON_UTILITY_H

#include <string>
#include <memory>
 //Externals includes
#include <json.hpp>
using json = nlohmann::json;

#include <SFML/System/Vector2.hpp>



namespace sfge
{
/**
* \brief Function that checks if the given json value is a number
*/
bool IsJsonValueNumeric(const json::value_type& jsonValue);
/**
* \brief Function that checks if the given parameter exists in the json
*/
bool CheckJsonExists(const json& jsonObject, std::string parameterName);
/**
* \brief Function that checks if the parameter exists and is of expected type
*/
bool CheckJsonParameter(const json& jsonObject, std::string parameterName, json::value_t expectedType);
/**
* \brief Function that checks if the parameters exists and is a number
*/
bool CheckJsonNumber(const json& jsonObject, std::string parameterName);
/**
* \brief Function that gets the Vector from an array or an object in the json
*/
sf::Vector2f GetVectorFromJson(const json& jsonObject, std::string parameterName);
/**
* \brief Function that loads a json file and returns a json object
*/
std::unique_ptr<json> LoadJson(std::string jsonPath);
}
#endif
