#ifndef SLAM3D_Neo4jGRAPH_HPP
#define SLAM3D_Neo4jGRAPH_HPP

#include <memory>
#include <string>
#include <vector>

#include <slam3d/core/Graph.hpp>
#include <slam3d/core/MeasurementStorage.hpp>




// #include "Neo4jQuery.hpp"
#include "Neo4jConversion.hpp"
#include "Neo4jConnection.hpp"
// #include "RedisMap.hpp"
// #include <boost/thread/shared_mutex.hpp>


// namespace web{ 
//     namespace http{ namespace client{ class http_client;}}
//     namespace json{ class value;}
// }


namespace slam3d {
    /**
     * @class Neo4jGraph
     * @brief Implementation of Graph using Neo4jGraphGraphLibrary.
     */
class Neo4jGraph : public Graph {
    public:

        //compatibility typedef
        typedef Neo4jConnection::ServerConfig Server;

        Neo4jGraph(Logger* log,
                    MeasurementStorage* storage,
                    const Neo4jConnection::ServerConfig &graphserver = Neo4jConnection::ServerConfig("127.0.0.1", 7687, "neo4j", "neo4j")
                    );
        ~Neo4jGraph();


        void init(const size_t &indexer_start = 0) override;

        /**
         * @brief delete the database contents
         * @details deletes all Nodes and edges from the graph
         * @return true if deletion was successful
         */
        void clear() override;

        /**
         * @brief 
         * @param id
         */
        const VertexObject getVertex(IdType id) const override;

        const VertexObject getVertex(boost::uuids::uuid id) const override;

        virtual void setVertex(IdType id, const VertexObject& v) override;

        /**
         * @brief 
         * @param source
         * @param target
         * @param sensor
         */
        const EdgeObject getEdge(IdType source, IdType target, const std::string& sensor) const override;

        /**
         * @brief Get all outgoing edges from given source.
         * @param source
         * @throw std::out_of_range
         */
        const EdgeObjectList getOutEdges(IdType source) const override;

		/**
		 * @brief Get a list of sensors of all vertices in the graph
		 * @return const std::set<std::string> list of all sensors within the graph
		 */
		virtual const std::set<std::string> getVertexSensors() const override;

		/**
		 * @brief Get a list of sensors of all vertices in the graph
		 * @return const std::set<std::string> list of all sensors within the graph
		 */
		virtual const std::set<std::string> getEdgeSensors() const override;

        /**
         * @brief Gets a list of all vertices from given sensors or all vertices when left out.
         * @param sensors
         */
        virtual const VertexObjectList getVertices(const StringSet& sensors = {}) const override;

		/**
		 * @brief Gets a list of all vertices with a given measurement type.
		 * @param sensor
		 */
		virtual const VertexObjectList getVerticesByType(const std::string& type) const override;

        /**
         * @brief Serch for nodes by using breadth-first-search
         * @param source start search from this node
         * @param range maximum number of steps to search from source
         */
        virtual const VertexObjectList getVerticesInRange(IdType source, unsigned range) const override;

		/**
		 * @brief Serch for nodes by location and radius
		 * @param location x,y,z location of the center
		 * @param radius the radius of the return
		 * @throw InvalidVertex
		 */
        virtual const VertexObjectList getNearbyVertices(const Transform &location, float radius, const StringSet& sensors = {}) const override;

        /**
         * @brief Gets a list of all edges from given sensor.
         * @param sensor
         */
        const EdgeObjectList getEdgesFromSensor(const std::string& sensor) const override;

        /**
         * @brief Get all connecting edges between given vertices.
         * @param vertices
         */
        const EdgeObjectList getEdges(const VertexObjectList& vertices) const override;

        /**
         * @brief Calculates the minimum number of edges between two vertices in the graph.
         * @param source
         * @param target
         */
        float calculateGraphDistance(IdType source, IdType target) const override;

        /**
         * @brief Write the current graph to a file (currently dot).
         * @details For larger graphs, this can take a very long time.
         * @param name filename without type ending
         */
        void writeGraphToFile(const std::string &name) override;


        void setCorrectedPose(IdType id, const Transform& pose) override;


    protected:
        /**
         * @brief Add the given VertexObject to the internal graph.
         * @param v
         */
        void addVertex(const VertexObject& v) override;

        /**
         * @brief Add the given EdgeObject to the internal graph.
         * @param e
         */
        // version to overload graph interface
        virtual void addEdge(const EdgeObject& e) override;
        // internal version
        virtual void addEdge(const EdgeObject& e, bool addInverse);

        /**
         * @brief 
         * @param source
         * @param target
         * @param sensor
         */
        virtual void removeEdge(IdType source, IdType target, const std::string& sensor) override;

        // /**
        //  * @brief Get a writable reference to a VertexObject.
        //  * @param id
        //  */
        // virtual VertexObject& getVertexInternal(IdType id);

        // /**
        //  * @brief Get a writable reference to an EdgeObject.
        //  * @param source
        //  * @param target
        //  * @param sensor
        //  */
        // virtual EdgeObject& getEdgeInternal(IdType source, IdType target, const std::string& sensor);

        // // /**
        //  * @brief
        //  * @param source
        //  * @param target
        //  * @param sensor
        //  */
        // OutEdgeIterator getEdgeIterator(IdType source, IdType target, const std::string& sensor) const;

    private:
        // std::string createQuery(const std::string& query, const web::json::value& params = web::json::value());

        std::shared_ptr<Neo4jConnection> neo4j;

        // neo4j_connection_t *connection;
        // The boost graph object
        // AdjacencyGraph mPoseGraph;

        // Mutex for graph access
        // mutable boost::shared_mutex mGraphMutex;

        // Index to map a vertex' id to its descriptor
        // IndexMap mIndexMap;

        // todo remove this and pass shared_ptr
        std::vector<VertexObject> vertexObjects;

        // std::shared_ptr<web::http::client::http_client> client;
        


        // todo replace with databae (value store)
        // std::map<std::string, Measurement::Ptr> measurements;

        // std::shared_ptr<RedisMap> measurements;
        // std::shared_ptr<MeasurementStorage> measurements;
};
}  // namespace slam3d

#endif
