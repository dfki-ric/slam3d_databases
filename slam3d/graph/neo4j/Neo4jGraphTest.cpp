#define BOOST_TEST_MODULE "Neo4jGraphTest"

#include <slam3d/core/FileLogger.hpp>
#include <slam3d/core/test_templates/GraphTest.hpp>

#include <slam3d/core/MeasurementStorage.hpp>
#include "Neo4jGraph.hpp"
#include "Neo4jConversion.hpp"

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#define private public
#define protected public

using namespace slam3d;

std::unique_ptr<Neo4jGraph> neo4jgraph;
slam3d::MeasurementStorage measurements;
Clock neo4jclock;
FileLogger neo4jlogger(neo4jclock, "neo4j_graph.log");



void initEigenTransform(slam3d::Transform* mat) {
    for (size_t c = 0; c < mat->matrix().cols(); ++c) {
        for (size_t r = 0; r < mat->matrix().rows(); ++r) {
            (*mat)(r, c) = r+c*mat->matrix().rows();
        }
    }
}

void initEigenMatrix(Eigen::MatrixXd* mat) {
    for (size_t c = 0; c < mat->matrix().cols(); ++c) {
        for (size_t r = 0; r < mat->matrix().rows(); ++r) {
            (*mat)(r, c) = r+c*mat->matrix().rows();
        }
    }
}

void initEigenVector(Eigen::Vector4f* mat) {
    // for (size_t c = 0; c < mat->matrix().cols(); ++c) {
        for (size_t r = 0; r < mat->matrix().rows(); ++r) {
            (*mat)(r) = r;
        }
    // }
}

void initEigenQuaternion(Eigen::Quaternionf* q) {
    q->x() = 1;
    q->y() = 2;
    q->z() = 3;
    q->w() = 4;
}

void initDB() {
    static bool initialized = false;
    if (!initialized) {
        neo4jlogger.setLogLevel(DEBUG);
        neo4jgraph = std::make_unique<Neo4jGraph>(&neo4jlogger);
        neo4jgraph->clear();
        initialized = true;
    }
}

BOOST_AUTO_TEST_CASE(neo4j_graph_construction) {
    initDB();
    test_graph_construction(neo4jgraph.get());
}

BOOST_AUTO_TEST_CASE(matrix_conversion) {
    initDB();
    slam3d::Transform t;
    initEigenTransform(&t);
    std::string ts = Neo4jConversion::eigenMatrixToString(t.matrix());
    slam3d::Transform tr(Eigen::Matrix4d(Neo4jConversion::eigenMatrixFromString(ts)));
    BOOST_CHECK_EQUAL(t.matrix(), tr.matrix());
}

BOOST_AUTO_TEST_CASE(contraint_conversion) {
    // neo4jgraph->co
}

BOOST_AUTO_TEST_CASE(measurement_storage) {
    initDB();

    PointCloud::Ptr cloud = PointCloud::Ptr(new PointCloud());

    cloud->push_back(slam3d::PointType(1, 2, 3));

    slam3d::Graph* g = dynamic_cast<slam3d::Graph*>(neo4jgraph.get());

    //todo add data to cloud

    PointCloudMeasurement::Ptr m = boost::make_shared<PointCloudMeasurement>(cloud);
    slam3d::MetaData meta = initMetaData(m->getTimestamp(), m->getTypeName(), "robot", "sensor", slam3d::Transform::Identity());

	slam3d::Transform tf = slam3d::Transform::Identity();
	slam3d::IdType id = g->addVertex(meta, tf);
	//BOOST_CHECK_EQUAL(id, exp_id);

	slam3d::VertexObject query_res;
	BOOST_CHECK_NO_THROW(query_res = g->getVertex(id));
	// BOOST_CHECK_EQUAL(query_res.index, exp_id);

    BOOST_CHECK_EQUAL(meta.robotName, query_res.measurement.robotName); 
    BOOST_CHECK_EQUAL(meta.sensorName, query_res.measurement.sensorName); 
    BOOST_CHECK_EQUAL(meta.typeName, query_res.measurement.typeName); 
}
