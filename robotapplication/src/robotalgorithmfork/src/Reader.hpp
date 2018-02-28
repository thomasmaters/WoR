/** 
 * @file   Reader.hpp
 * @author Peter van Leeuwen
 * @author Mustafa Sabur
 * @date   February 20, 2017, 12:18 PM
 */

#ifndef BATCHREADER_HPP
#define BATCHREADER_HPP

#include <iostream>
#include <vector>
#include "Specification.hpp"


/**
 * @brief Configuration reading and writing.
 */
struct Reader
{
    /**
     * Reads a batch file where a specification is written per line.
     * @param filename The filename of the file that contains the specifications which needs be loaded.
     * @return Vector with read specifications.
     */
    std::vector<Specification> readBatchfile(const std::string& filename);
    
    /**
     * Reads the user input.
     * @param is The istream that contains the user input.
     * @param line The string where the read input is put in.
     * @return Returns true when the input is read, returns false when the input is empty.
     */
    bool readUserInput(std::istream& is, std::string& line); 
    bool writeBatchOutput(const std::string& filename, const std::string& data);

};

#endif /* BATCHREADER_HPP */