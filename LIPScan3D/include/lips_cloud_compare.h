#pragma once

#ifdef LIPSCLOUDCOMPARE_EXPORTS
#define LIPSCLOUDCOMPARE_API __declspec(dllexport)
#else
#define LIPSCLOUDCOMPARE_API __declspec(dllimport)
#endif

namespace lips
{
	typedef void* HandleTriMeshModel;
	//handle to triangle mesh data defined in LIPScan3D SDK

	class LIPSccGenericCloud;   //Implements of CCCoreLib::GenericIndexedCloudPersist
	class LIPSccGenericMesh;    //Implements of CCCoreLib::GenericIndexedMesh
	class LIPSregistrationTools;//Applies ICP registration on two entities

	struct LIPSCLOUDCOMPARE_API VERTEX3F
	{
		float x;
		float y;
		float z;
		float u[3];
	};
	struct LIPSCLOUDCOMPARE_API COLOR3UC
	{
		unsigned char r;
		unsigned char g;
		unsigned char b;
	};
	typedef std::map<float, COLOR3UC> LIPSCLOUDCOMPARE_API ColorScale;
	
	struct LIPSCLOUDCOMPARE_API ColorScaleBar
	{
		int ImgWitdh;
		int ImgHeight;
		unsigned char* Img;
		int TextNum;
		SIZE* TextSize;
		unsigned char** TextImg;
	};

	extern "C" LIPSCLOUDCOMPARE_API bool lipsApplyTransformMesh(HandleTriMeshModel mesh, double* R, float* T, float* C, float* S);

	typedef void* HandleMeshComparisonContext;
	//Handle to member variables in MeshComparison class

	class LIPSCLOUDCOMPARE_API MeshComparison
	{
	public:
		MeshComparison();
		~MeshComparison();

	private:
		//! Handle to member variables
		HandleMeshComparisonContext m_context;

#pragma region Enable License
	public:
		bool Init(const char*);
		bool IsEnable();
#pragma endregion
#pragma region Load Mesh
	public:
		bool setCompMesh(HandleTriMeshModel);
		bool loadCompMesh(const char*);
		bool loadRefMesh(const char*);
		HandleTriMeshModel getCompMesh();
		HandleTriMeshModel getRefMesh();
#pragma endreiogn
#pragma region Aling Mesh 
	public:
		//! Adds a point to the 'align' set
		void addAlignedPoint(float x, float y, float z);
		//! Adds a point to the 'reference' set
		void addReferencePoint(float x, float y, float z);
		//! Clear the 'align' set
		void clearAlignedPoints(void);
		//! Clear the 'reference' set
		void clearRefPoints(void);
		//! Get the 'align' set
		std::vector<VERTEX3F> getAlignedPoints(void);
		//! Get the 'reference' set
		std::vector<VERTEX3F> getReferencePoints(void);
		//! Matching point pairs
		void Align();
#pragma endregion
#pragma region Mesh Registration
	public:
		//! Fine registration(ICP)
		bool Register(float, int, bool);
#pragma endregion
#pragma region Mesh Distances
	public:
		bool ComputeMeshAproxDistance();
		bool ComputeMeshDistances(bool);
		bool GenCompMeshDistanceColors(std::string, float);
		void GetBBoxInfos();
	private:
		bool isValid();
		int getBestOctreeLevel();
		int determineBestOctreeLevel(double);
#pragma endregion
#pragma region Histogram
	public:
		//! Dynamically computes histogram bins from scalar field
		void GenHistogram(size_t);
#pragma endregion
#pragma region Export File
	public:
		//! Exports histogram to a CSV file
		bool ExportToCSV(const char*);
#pragma endregion
#pragma region ColorScale Setting
	public:
		bool AddColorScale(std::string);
		unsigned int GetColorScaleSize();
		ColorScale* GetColorScale(std::string);
		bool AddColorStep(std::string, float , unsigned char , unsigned char , unsigned char );
		void ClearColorScale();
#pragma endregion
#pragma region Generate Histogram Image, Colorbar & Text
	public:
		// Generate Histogram image buffer
		unsigned char* GenHistogramImg(int&, int&);

		// Generate Histogram colorbar buffer
		unsigned char* GenColorbarBuf(std::string, int&, int&);

		// Geberate Histogram colorbar text buffer
		unsigned char** GenColorbarText(float, int, SIZE*, bool);
		unsigned char** GenColorbarText(float, std::string, SIZE*);
#pragma endregion
#pragma region Reset
	public:
		// Reset Compared entity and Referenced entity.
		void Reset();
#pragma endregion
	};
}