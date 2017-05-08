#ifndef TENSORFLOW_MY_TENSORFLOW_MODEL_RUN_LIB_H_
#define TENSORFLOW_MY_TENSORFLOW_MODEL_RUN_LIB_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
// implementation of the class in the external library: lib_tensorflow_model_run.so

// forward declarations
namespace tensorflow {
	class Session;
	class GraphDef;
}


// Load a trained tensorflow model
// hold the model in a session
// make predictions with it
class MyPredict {
	public:
		MyPredict();
		virtual ~MyPredict();
		void init(std::string modelPath); // takes a while bc loads the graph from a file
		std::vector<float> modelPrediction(std::vector<float> pointsFlat, std::vector<float> zmpvec, int dataWidth, int dataHeight);
		void close();

	private:
		tensorflow::Session* session;
};


#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif  // TENSORFLOW_MY_TENSORFLOW_MODEL_RUN_LIB_H_

