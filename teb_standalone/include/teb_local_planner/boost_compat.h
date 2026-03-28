/*********************************************************************
 *
 * Boost compatibility header for teb_local_planner standalone build.
 *
 * This header ensures all required Boost headers are included before
 * any teb_local_planner headers. It is force-included via compiler
 * flags so that every translation unit picks up the necessary
 * definitions (e.g. boost::prior, boost::optional).
 *
 *********************************************************************/

#ifndef TEB_BOOST_COMPAT_H_
#define TEB_BOOST_COMPAT_H_

// boost::prior / boost::next  (used in timed_elastic_band.hpp, graph_search.cpp)
#include <boost/next_prior.hpp>

// boost::optional / boost::none  (used in timed_elastic_band.h/hpp, homotopy_class_planner.h/hpp, h_signature.h)
#include <boost/optional.hpp>

#endif // TEB_BOOST_COMPAT_H_
