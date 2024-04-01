#include <octomap/octomap.h>
#include <octomap/OcTree.h>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input.bt> <output.bt>\n";
        return 1;
    }

    std::string input_filename(argv[1]);
    std::string output_filename(argv[2]);

    // Load the .bt file into an OcTree
    octomap::OcTree* tree = new octomap::OcTree(input_filename);

    // Traverse the tree
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end=tree->end_leafs(); it!= end; ++it) {
        // Check the coordinates of each node
        if(it.getX() < -29 || it.getX() > 29 || it.getY() < -29 || it.getY() > 29 || it.getZ() < -1 || it.getZ() > 15) {
            // Delete the node if it's outside the specified range
            tree->deleteNode(it.getKey());
        }
    }

    // Save the modified tree back to a .bt file
    tree->writeBinary(output_filename);

    delete tree;

    return 0;
}
