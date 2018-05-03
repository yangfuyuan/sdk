/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  ydlidar_math_kmeans_H
#define  ydlidar_math_kmeans_H


#include <assert.h>
#include <stddef.h>
#include <vector>
#include <deque>
#include <list>
#include <map>
#include <stdint.h>
#include <string.h>


namespace ydlidar
{
	namespace math
	{

    /** CArrayNumeric is an array for numeric types supporting several mathematical operations (actually, just a wrapper on std::array<T,N>)
      * \sa CArrayFloat, CArrayDouble, CArray
      */
    template <typename T, std::size_t N>
    class CArrayNumeric : public std::array<T,N>
    {
        public:
        typedef T                    value_type;
        typedef std::array<T,N> Base;


        CArrayNumeric() {}  //!< Default constructor
        /** Constructor from initial values ptr[0]-ptr[N-1] */
        CArrayNumeric(const T*ptr) :std::array<T,N>(ptr) {}

        /** Initialization from a vector-like source, that is, anything implementing operator[]. */
        template <class ARRAYLIKE>
        explicit CArrayNumeric(const ARRAYLIKE &obj) :std::array<T,N>(obj) {}

    };

    // --------------  Partial specializations of CArrayNumeric -----------

    /** A partial specialization of CArrayNumeric for float numbers.
      * \sa CArrayNumeric, CArray */
    template <std::size_t N>
    class CArrayFloat : public CArrayNumeric<float,N>
    {
    public:
        typedef CArrayNumeric<float,N> Base;
        typedef CArrayFloat<N> mrpt_autotype;

        CArrayFloat() {}  //!< Default constructor
        CArrayFloat(const float*ptr) : CArrayNumeric<float,N>(ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]


        /** Initialization from a vector-like source, that is, anything implementing operator[]. */
        template <class ARRAYLIKE>
        explicit CArrayFloat(const ARRAYLIKE &obj) : CArrayNumeric<float,N>(obj) {}
    };

    /** A partial specialization of CArrayNumeric for double numbers.
      * \sa CArrayNumeric, CArray */
    template <std::size_t N>
    class CArrayDouble : public CArrayNumeric<double,N>
    {
    public:
        typedef CArrayNumeric<double,N> Base;
        typedef CArrayDouble<N> mrpt_autotype;

        CArrayDouble() {}  //!< Default constructor
        CArrayDouble(const double*ptr) : CArrayNumeric<double,N>(ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]


        /** Initialization from a vector-like source, that is, anything implementing operator[]. */
        template <class ARRAYLIKE>
        explicit CArrayDouble(const ARRAYLIKE &obj) : CArrayNumeric<double,N>(obj) {}
    };

    /** A partial specialization of CArrayNumeric for int numbers.
      * \sa CArrayNumeric, CArray */
    template <std::size_t N>
    class CArrayInt : public CArrayNumeric<int,N>
    {
    public:
        typedef CArrayNumeric<int,N> Base;
        typedef CArrayInt<N> mrpt_autotype;

        CArrayInt() {}  //!< Default constructor
        CArrayInt(const int*ptr) : CArrayNumeric<int,N>(ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]
    };

    /** A partial specialization of CArrayNumeric for unsigned int numbers.
      * \sa CArrayNumeric, CArray */
    template <std::size_t N>
    class CArrayUInt : public CArrayNumeric<unsigned int,N>
    {
    public:
        typedef CArrayNumeric<unsigned int,N> Base;
        typedef CArrayUInt<N> mrpt_autotype;

        CArrayUInt() {}  //!< Default constructor
        CArrayUInt(const unsigned int*ptr) : CArrayNumeric<unsigned int,N>(ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]
    };

    // Fwrd. decl:
    template<class T> class aligned_allocator;


    /** Helper types for STL containers with std memory allocators.  */
    template <class TYPE1,class TYPE2=TYPE1>
    struct aligned_containers
    {
        typedef std::pair<TYPE1,TYPE2> pair_t;
        typedef std::vector<TYPE1, std::allocator<TYPE1> > vector_t;
        typedef std::deque<TYPE1,  std::allocator<TYPE1> > deque_t;
        typedef std::list<TYPE1, std::allocator<TYPE1> > list_t;
        typedef std::map<TYPE1,TYPE2,std::less<TYPE1>, std::allocator<std::pair<const TYPE1,TYPE2> > > map_t;
        typedef std::multimap<TYPE1,TYPE2,std::less<TYPE1>, std::allocator<std::pair<const TYPE1,TYPE2> > > multimap_t;
    };

		namespace detail {
			// Auxiliary method: templatized for working with float/double's.
			template <typename SCALAR>
            double  internal_kmeans(
				const bool use_kmeansplusplus_method,
				const size_t nPoints,
				const size_t k,
				const size_t dims,
				const SCALAR *points,
				const size_t attempts,
				SCALAR* out_center,
				int *out_assignments
				);

			// Auxiliary method, the actual code of the two front-end functions offered to the user below.
			template <class LIST_OF_VECTORS1,class LIST_OF_VECTORS2>
			double stub_kmeans(
				const bool use_kmeansplusplus_method,
				const size_t k,
				const LIST_OF_VECTORS1 & points,
				std::vector<int>  &assignments,
				LIST_OF_VECTORS2 *out_centers,
				const size_t attempts
				)
			{
                assert(k>=1);

				const size_t N = points.size();
				assignments.resize(N);
				if (out_centers) out_centers->clear();
				if (!N)
					return 0;	// No points, we're done.
				// Parse to required format:
				size_t dims=0;
				const typename LIST_OF_VECTORS1::const_iterator it_first=points.begin();
				const typename LIST_OF_VECTORS1::const_iterator it_end  =points.end();
                typedef typename LIST_OF_VECTORS1::value_type TInnerVector;
                typedef typename LIST_OF_VECTORS2::value_type TInnerVectorCenters;

                std::vector<typename TInnerVector::value_type> raw_vals;
                typename TInnerVector::value_type *trg_ptr=NULL;
				for (typename LIST_OF_VECTORS1::const_iterator it=it_first;it!=it_end;++it)
				{
					if (it==it_first)
					{
						dims = it->size();
                        assert(dims>0);
						raw_vals.resize(N*dims);
						trg_ptr = &raw_vals[0];
					}
					else
					{
                        assert(size_t(dims)==size_t(it->size()));
					}

                    ::memcpy(trg_ptr, &(*it)[0], dims*sizeof(typename TInnerVector::value_type));
					trg_ptr+=dims;
				}
				// Call the internal implementation:
                std::vector<typename TInnerVectorCenters::value_type> centers(dims*k);
				const double ret = detail::internal_kmeans(false,N,k,points.begin()->size(),&raw_vals[0],attempts,&centers[0],&assignments[0]);
				// Centers:
				if (out_centers)
				{
                    const typename TInnerVectorCenters::value_type *center_ptr = &centers[0];
					for (size_t i=0;i<k;i++)
					{
                        TInnerVectorCenters c;
                        for (size_t j=0;j<dims;j++) {
                            if(j < c.size())
                                c[j] = *center_ptr++;
                        }
                        out_centers->push_back(c);
					}
				}
				return ret;
			}
		} // end detail namespace


		/** @name k-means algorithms
		@{ */

		/** k-means algorithm to cluster a list of N points of arbitrary dimensionality into exactly K clusters.
		  *   The list of input points can be any template CONTAINER<POINT> with:
		  *		- CONTAINER can be: Any STL container: std::vector,std::list,std::deque,...
		  *		- POINT can be:
		  *			- std::vector<double/float>
		  *			- CArrayDouble<N> / CArrayFloat<N>
		  *
		  *  \param k [IN] Number of cluster to look for.
		  *  \param points [IN] The list of N input points. It can be any STL-like containers of std::vector<float/double>, for example a std::vector<mrpt::math::CVectorDouble>, a std::list<CVectorFloat>, etc...
		  *  \param assignments [OUT] At output it will have a number [0,k-1] for each of the N input points.
		  *  \param out_centers [OUT] If not NULL, at output will have the centers of each group. Can be of any of the supported types of "points", but the basic coordinates should be float or double exactly as in "points".
		  *  \param attempts [IN] Number of attempts.
		  *
		  * \sa A more advanced algorithm, see: kmeanspp
		  * \note Uses the kmeans++ implementation by David Arthur (2009, http://www.stanford.edu/~darthur/kmpp.zip).
		  */
		template <class LIST_OF_VECTORS1,class LIST_OF_VECTORS2>
		inline double kmeans(
			const size_t k,
			const LIST_OF_VECTORS1 & points,
			std::vector<int>  &assignments,
			LIST_OF_VECTORS2 *out_centers = NULL,
			const size_t attempts = 3
			)
		{
			return detail::stub_kmeans(false /* standard method */, k,points,assignments,out_centers,attempts);
		}

		/** k-means++ algorithm to cluster a list of N points of arbitrary dimensionality into exactly K clusters.
		  *   The list of input points can be any template CONTAINER<POINT> with:
		  *		- CONTAINER can be: Any STL container: std::vector,std::list,std::deque,...
		  *		- POINT can be:
		  *			- std::vector<double/float>
		  *			- CArrayDouble<N> / CArrayFloat<N>
		  *
		  *  \param k [IN] Number of cluster to look for.
		  *  \param points [IN] The list of N input points. It can be any STL-like containers of std::vector<float/double>, for example a std::vector<mrpt::math::CVectorDouble>, a std::list<CVectorFloat>, etc...
		  *  \param assignments [OUT] At output it will have a number [0,k-1] for each of the N input points.
		  *  \param out_centers [OUT] If not NULL, at output will have the centers of each group. Can be of any of the supported types of "points", but the basic coordinates should be float or double exactly as in "points".
		  *  \param attempts [IN] Number of attempts.
		  *
		  * \sa The standard kmeans algorithm, see: kmeans
		  * \note Uses the kmeans++ implementation by David Arthur (2009, http://www.stanford.edu/~darthur/kmpp.zip).
		  */
		template <class LIST_OF_VECTORS1,class LIST_OF_VECTORS2>
		inline double kmeanspp(
			const size_t k,
			const LIST_OF_VECTORS1 & points,
			std::vector<int>  &assignments,
			LIST_OF_VECTORS2 *out_centers = NULL,
			const size_t attempts = 3
			)
		{
			return detail::stub_kmeans(true /* kmeans++ algorithm*/, k,points,assignments,out_centers,attempts);
		}

		/** @} */

	} // End of MATH namespace
} // End of namespace
#endif
