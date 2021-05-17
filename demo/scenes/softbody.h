
class Cutter {
public:
	Vec3 cutterPos;
	Matrix33 rotMat;
	Matrix44 basis;
	Matrix44 basisInv;

	float cutterRot;

	// cluster indices
	int original = -1;
	int front = -1;
	int back = -1;

	Cutter() {
		cutterPos.Set(0, 0, 0);

		rotMat = rotMat.Identity();

		basis.SetAxis(0, Vec3(1, 0, 0));
		basis.SetAxis(1, Vec3(0, 1, 0));
		basis.SetAxis(2, Vec3(0, 0, 1));
		basis.SetTranslation(Point3(cutterPos.x, cutterPos.y, cutterPos.z));

		basisInv = AffineInverse(basis);
	}

	void Update() {
		basis.SetAxis(0, Vec3(rotMat.cols[0]));
		basis.SetAxis(1, Vec3(rotMat.cols[1]));
		basis.SetAxis(2, Vec3(rotMat.cols[2]));
		basis.SetTranslation(Point3(cutterPos.x, cutterPos.y, cutterPos.z));

		basisInv = AffineInverse(basis);

		 //cout << basis.columns[0][0] << " " << basis.columns[1][0] << " " << basis.columns[2][0] << " " << basis.columns[3][0] << endl;
		 //cout << basis.columns[0][1] << " " << basis.columns[1][1] << " " << basis.columns[2][1] << " " << basis.columns[3][1] << endl;
		 //cout << basis.columns[0][2] << " " << basis.columns[1][2] << " " << basis.columns[2][2] << " " << basis.columns[3][2] << endl;
		 //cout << basis.columns[0][3] << " " << basis.columns[1][3] << " " << basis.columns[2][3] << " " << basis.columns[3][3] << endl;
	}

	void Rotate(float x, float y, float z) {
		rotMat.MatRotate(x, y, z);
	}

	void Reset() {
		original = -1;
		front = -1;
		back = -1;
	}
};

class OneCluster {
public:
	vector<int> indices;
	float hue;

	OneCluster(float _hue) { hue = _hue; }

	bool isInCluster(int particleIndex) {
		return (std::find(indices.begin(), indices.end(), particleIndex) != indices.end());
	}
};

class Clusters {
public:
	vector<OneCluster> clusterArray;
	vector<Vec3> points;

	vector<float> hues;
	float gap = 0.1;
	bool tipDifferent;

	vector<int> indices;
	vector<int> offsets;
	vector<float> coefficients;

	Clusters() {
		Init();
		RandInit();
	}

	void RemoveEmptyCluster() {
		for (size_t i = 0; i < clusterArray.size(); i++)
		{
			if (clusterArray[i].indices.size() == 0)
			{
				clusterArray.erase(clusterArray.begin() + i);
			}
		}
	}

	void SplitParticles(Cutter& cutter, bool isCutting) {
		if (!isCutting || tipDifferent)
		{
			cutter.Reset();
			RemoveEmptyCluster();
			tipDifferent = false;
			return;
		}

		// TODO: detect if region 0 has any particles

		// detect if there is a contact
		if (cutter.original == -1)
		{
			for (size_t i = 0; i < points.size(); i++)
			{
				Vec3 p = Vec3(points[i]);
				Vec3 newP = (cutter.basisInv * (Point3)p);
				//cout << p.dist(newP) << endl;

				if (newP.y < 0 && newP.y > -gap && p.dist(cutter.cutterPos) < gap)
				{
					// set the original index
					cutter.original = inWhichCluster(i);

					// create new cluster and store ID in front and back
					OneCluster oneCluster(Randf());
					clusterArray.push_back(oneCluster);
					cutter.front = clusterArray.size() - 1;

					OneCluster oneCluster2(Randf());
					clusterArray.push_back(oneCluster2);
					cutter.back = clusterArray.size() - 1;

					return;
				}
			}
		}

		// skip if there is no contact yet
		if (cutter.original == -1)
		{
			return;
		}

		cout << "contact!" << endl;
		// start cutting after contact

		// region graph
		// ______|______
		// |__4__|__3__|
		// |__2__|__1__|
		// |_____0_____|

		for (size_t i = 0; i < points.size(); i++)
		{
			Vec3 p = Vec3(points[i]);
			Vec3 newP = (cutter.basisInv * (Point3)p);

			// step -1: keep check region 0, switch restart cutting when particle in region 0 belongs to different clusters
			// (other than cutter.original)
			if (newP.y < -gap && newP.y > -gap * 2 && p.dist(cutter.cutterPos) < gap * 2)
			{
				// cout << "cutter.original" << cutter.original << " inWhichCluster: " << inWhichCluster(i) << endl;
				if (cutter.original != inWhichCluster(i))
				{
					cout << "Re-Cut !!!!" << endl;
					tipDifferent = true;
					break;
				}
			}

			// step 0: check if particle in region 1,2,3,4 belongs to some clusters
			// if particle contain in original cluster, keep calculating
			// otherwise, just pass this particle
			if (newP.y > -gap)
			{
				vector<int> inWhich = inWhichClusters(i);
				int ori = cutter.original;
				bool inOriginal = (std::find(inWhich.begin(), inWhich.end(), ori) != inWhich.end());
				if (inOriginal == false)
				{
					continue;
				}
				// step 1: For region 1,2
				// remove particles away from it's all original clusters
				// add particles:
				// (1) cutter.front, cutter.original.  (2) cutter.back, cutter,original

				if (newP.y < 0)
				{
					if (newP.x > 0)
					{
						RemoveFromAllClusters(i);
						AddToClusters(i, cutter.original);
						AddToClusters(i, cutter.front);
					}
					else if (newP.x <= 0)
					{
						RemoveFromAllClusters(i);
						AddToClusters(i, cutter.original);
						AddToClusters(i, cutter.back);
					}
				}

				// step 2: For region 3,4
				// remove particles from cutter.original
				// add particles to cutter.front, cutter.back
				if (newP.y >= 0)
				{
					RemoveFromAllClusters(i);
					if (newP.x > 0)
					{
						AddToClusters(i, cutter.front);
					}
					else if (newP.x <= 0)
					{
						AddToClusters(i, cutter.back);
					}
				}
			}
		}
	}

	void UpdateData() {
		// update rigidIndices and rigidOffsets
		Init();
		for (size_t i = 0; i < clusterArray.size(); i++)
		{
			for (size_t j = 0; j < clusterArray[i].indices.size(); j++)
			{
				indices.push_back(clusterArray[i].indices[j]);
				hues[clusterArray[i].indices[j]] = clusterArray[i].hue;
			}
			offsets.push_back(indices.size());
			coefficients.push_back(0.05);
		}
	}

	void Init() {
		indices.clear();
		offsets.clear();
		offsets.push_back(0);
		coefficients.clear();
	}

	void Restart() {
		Init();
		tipDifferent = false;
		points.clear();
		indices.clear();
		offsets.clear();
		clusterArray.clear();

		// Initilize positions, indices and offsets in clusters
		OneCluster c1(0.5f);

		for (size_t i = 0; i < g_buffers->positions.size(); i++)
		{
			points.push_back(Vec3(g_buffers->positions[i]));
			hues.push_back(0.5f);
			c1.indices.push_back(i);
		}
		clusterArray.push_back(c1);

		for (size_t i = 0; i < g_buffers->rigidIndices.size(); i++)
		{
			indices.push_back(g_buffers->rigidIndices[i]);
		}
		for (size_t i = 0; i < g_buffers->rigidOffsets.size(); i++)
		{
			offsets.push_back(g_buffers->rigidOffsets[i]);
		}
	}

	int inWhichCluster(int particleIndex) {
		int clusterIndex = -1;
		for (size_t i = 0; i < clusterArray.size(); i++)
		{
			if (clusterArray[i].isInCluster(particleIndex))
			{
				clusterIndex = i;
			}
		}
		return clusterIndex;
	}

	vector<int> inWhichClusters(int particleIndex) {
		vector<int> result;
		for (size_t i = 0; i < clusterArray.size(); i++)
		{
			if (clusterArray[i].isInCluster(particleIndex))
			{
				result.push_back(i);
			}
		}
		return result;
	}

	void RemoveFromAllClusters(int particleIndex) {

		vector<int> inWhich = inWhichClusters(particleIndex);

		for (size_t i = 0; i < inWhich.size(); i++)
		{
			vector<int>::iterator iBegin = clusterArray[inWhich[i]].indices.begin();
			vector<int>::iterator iEnd = clusterArray[inWhich[i]].indices.end();

			clusterArray[inWhich[i]].indices.erase(
				std::remove(iBegin, iEnd, particleIndex), iEnd);
		}
	}

	void AddToClusters(int particleIndex, int clusterId) {
		if (!(std::find(clusterArray[clusterId].indices.begin(), clusterArray[clusterId].indices.end(), particleIndex) != clusterArray[clusterId].indices.end()))
		{
			clusterArray[clusterId].indices.push_back(particleIndex);
		}
	}
};

class SoftBody : public Scene {
public:
	bool flag = false;
	vector<Vec3> originalRestPos;
	vector<Vec3> originalPos;

	Cutter cutter;
	Clusters clusters;

	Mesh* m;

	SoftBody(const char* name, const char* mesh) : Scene(name),
		mFile(mesh),
		mScale(2.0f),
		mOffset(0.0f, 1.0f, 0.0f),
		mRadius(0.1f),
		mClusterSpacing(1.0f),
		mClusterRadius(0.0f),
		mClusterStiffness(0.5f),
		mLinkRadius(0.0f),
		mLinkStiffness(1.0f),
		mGlobalStiffness(0.0f),
		mSurfaceSampling(0.0f),
		mVolumeSampling(4.0f),
		mSkinningFalloff(2.0f),
		mSkinningMaxDistance(100.0f),
		mPlasticThreshold(0.0f),
		mPlasticCreep(0.0f),
		mRelaxationFactor(1.0f),
		mPlinth(false) {
		mStack[0] = 1;
		mStack[1] = 1;
		mStack[2] = 1;
	}

	virtual void Initialize() {

		m = ImportMesh(GetFilePathByPlatform("../../data/knife.obj").c_str());
		m->Normalize();

		float radius = mRadius;

		g_params.radius = radius;
		g_params.dynamicFriction = 0.35f;
		g_params.particleFriction = 0.25f;
		g_params.dissipation = 0.0f;
		g_params.numIterations = 4;
		g_params.viscosity = 0.0f;
		g_params.drag = 0.0f;
		g_params.lift = 0.0f;
		g_params.collisionDistance = radius * 0.75f;

		g_params.plasticThreshold = mPlasticThreshold;
		g_params.plasticCreep = mPlasticCreep;

		g_params.relaxationFactor = mRelaxationFactor;

		g_windStrength = 0.0f;

		g_numSubsteps = 2;

		// draw options
		g_drawPoints = true;
		g_wireframe = false;
		g_drawSprings = false;
		g_drawBases = false;
		g_drawMesh = false;

		g_buffers->rigidOffsets.push_back(0);

		mInstances.resize(0);

		CreateBodies();

		// update rigid local positions
		if (g_buffers->rigidOffsets.size())
		{
			assert(g_buffers->rigidOffsets.size() > 1);

			const int numRigids = g_buffers->rigidOffsets.size() - 1;

			// calculate local rest space positions
			g_buffers->rigidLocalPositions.resize(g_buffers->rigidOffsets.back());
			CalculateRigidLocalPositions(&g_buffers->positions[0], g_buffers->positions.size(), &g_buffers->rigidOffsets[0], &g_buffers->rigidIndices[0], numRigids, &g_buffers->rigidLocalPositions[0]);

			g_buffers->rigidRotations.resize(g_buffers->rigidOffsets.size() - 1, Quat());
			g_buffers->rigidTranslations.resize(g_buffers->rigidOffsets.size() - 1, Vec3());
		}

		originalPos.clear();
		originalRestPos.clear();

		cout << "rigidLocalPositions: " << g_buffers->rigidLocalPositions.size() << endl;
		for (size_t i = 0; i < g_buffers->rigidLocalPositions.size(); i++)
		{
			originalRestPos.push_back(Vec3(g_buffers->rigidLocalPositions[i]));
		}
		for (size_t i = 0; i < g_buffers->positions.size(); i++)
		{
			originalPos.push_back(Vec3(g_buffers->positions[i]));
		}

		// Initialize cluster data
		clusters.Restart();

		if (mPlinth)
		{
			AddPlinth();
		}

		// fix any particles below the ground plane in place
		for (int i = 0; i < int(g_buffers->positions.size()); ++i)
			if (g_buffers->positions[i].y < 0.0f)
				g_buffers->positions[i].w = 0.0f;

		// expand radius for better self collision
		g_params.radius *= 1.5f;

		g_lightDistance *= 1.5f;
	}

	virtual void CreateBodies() {
		// build soft body
		for (int x = 0; x < mStack[0]; ++x)
		{
			for (int y = 0; y < mStack[1]; ++y)
			{
				for (int z = 0; z < mStack[2]; ++z)
				{
					CreateSoftBody(mRadius, mOffset + Vec3(x * (mScale.x + 1), y * (mScale.y + 1), z * (mScale.z + 1)) * mRadius, mClusterStiffness, mInstances.size());
				}
			}
		}
	}
	float xRot = 0, yRot = 0, zRot = 0;

	virtual void KeyDown(int key) {
		if (key == 32)
		{
			flag = !flag;
		}

	}

	virtual void Update() {
		// update particle positions into clusters
		for (size_t i = 0; i < g_buffers->positions.size(); i++)
		{
			clusters.points[i] = Vec3(g_buffers->positions[i]);
		}

		Vec3 origin, dir;
		GetViewRay(g_lastx, g_screenHeight - g_lasty, origin, dir);
		dir *= 5;

		// set cutter position
		cutter.Rotate(xRot, yRot, zRot);
		cutter.cutterPos.Set((origin + dir).x, (origin + dir).y, (origin + dir).z);

		if (flag)
		{
			// update cutting plane and matrix
			cutter.Update();
			clusters.SplitParticles(cutter, g_lastb == SDL_BUTTON_LEFT);
		}
		clusters.UpdateData();

		//if (!flag) {
		//	return;
		//}
		//flag = false;

		//// copy the positions, rigidIndices, rigidOffsets, from g_buffers

		//vector<int> indices;
		//vector<int> offsets;
		//offsets.push_back(0);
		//vector<Vec3> clusterCenters;

		//g_buffers->rigidCoefficients.resize(0);

		//for (size_t i = 0; i < g_buffers->rigidOffsets.size(); i++)
		//{
		//	g_buffers->rigidOffsets[i] = g_buffers->rigidOffsets[i];
		//	cout << g_buffers->rigidOffsets[i] << endl;
		//}

		//Vec3 mean_vec;
		//for (size_t i = 0; i < g_buffers->positions.size(); i++)
		//{
		//	mean_vec += Vec3(g_buffers->positions[i]);
		//}
		//mean_vec /= g_buffers->positions.size();

		//// cluster 1
		//Vec3 center1;
		//int num1;
		//for (size_t i = 0; i < g_buffers->positions.size(); i++)
		//{
		//	Vec4 p = g_buffers->positions[i];
		//	if (p.y < mean_vec.y) {
		//		g_buffers->densities[i] = 0.1;
		//		indices.push_back(i);
		//		center1 += Vec3(p);
		//		num1++;
		//	}
		//}
		//center1 /= num1;
		//offsets.push_back(indices.size());
		//g_buffers->rigidCoefficients.push_back(0.1);

		//// cluster 2
		//Vec3 center2;
		//int num2;
		//for (size_t i = 0; i < g_buffers->positions.size(); i++)
		//{
		//	Vec4 p = g_buffers->positions[i];
		//	if (p.y >= mean_vec.y) {
		//		g_buffers->densities[i] = 0.5;
		//		indices.push_back(i);
		//		center2 += Vec3(p);
		//		num2++;
		//	}
		//}
		//center2 /= num2;
		//offsets.push_back(indices.size());
		//g_buffers->rigidCoefficients.push_back(0.1);

		//g_buffers->rigidIndices.assign(&indices[0], indices.size());
		//g_buffers->rigidOffsets.assign(&offsets[0], offsets.size());

		// cout << "---------------------------------------------" << endl;
		// cout << "original: " << cutter.original << endl;
		// cout << "front: " << cutter.front << endl;
		// cout << "back: " << cutter.back << endl;

		//cout << "clusters.indices: " << clusters.indices.size() << endl;
		//cout << "clusters.offsets: " << clusters.offsets.size() << endl;

		if (g_lastb != SDL_BUTTON_LEFT)
		{
			g_buffers->rigidIndices.assign(&clusters.indices[0], clusters.indices.size());
			g_buffers->rigidOffsets.assign(&clusters.offsets[0], clusters.offsets.size());
			g_buffers->rigidCoefficients.resize(clusters.coefficients.size());
			g_buffers->rigidCoefficients.assign(&clusters.coefficients[0], clusters.coefficients.size());
		}
		g_buffers->densities.resize(clusters.hues.size());
		g_buffers->densities.assign(&clusters.hues[0], clusters.hues.size());

		// builds rigids constraints
		if (g_buffers->rigidOffsets.size())
		{
			assert(g_buffers->rigidOffsets.size() > 1);

			const int numRigids = g_buffers->rigidOffsets.size() - 1;

			// calculate local rest space positions
			g_buffers->rigidLocalPositions.resize(g_buffers->rigidOffsets.back());

			//CalculateRigidLocalPositions(&g_buffers->positions[0], g_buffers->positions.size(), &g_buffers->rigidOffsets[0], &g_buffers->rigidIndices[0], numRigids, &g_buffers->rigidLocalPositions[0]);
			UpdateRigidLocalPositions(&originalPos[0], g_buffers->positions.size(), &g_buffers->rigidOffsets[0], &g_buffers->rigidIndices[0], numRigids, &g_buffers->rigidLocalPositions[0]);

			g_buffers->rigidRotations.resize(g_buffers->rigidOffsets.size() - 1, Quat());
			g_buffers->rigidTranslations.resize(g_buffers->rigidOffsets.size() - 1, Vec3());
		}

		//cout << "rigidOffsets: " << g_buffers->rigidOffsets.size() << endl;
		//cout << "rigidIndices: " << g_buffers->rigidIndices.size() << endl;
		//cout << "rigidLocalPositions: " << g_buffers->rigidLocalPositions.size() << endl;
		//cout << "rigidLocalNormals: " << g_buffers->rigidLocalNormals.size() << endl;
		//cout << "rigidCoefficients: " << g_buffers->rigidCoefficients.size() << endl;
		//cout << "rigidRotations: " << g_buffers->rigidRotations.size() << endl;
		//cout << "rigidTranslations: " << g_buffers->rigidTranslations.size() << endl;
	}

	// Mobai Add
	virtual void Sync() {

		NvFlexSetRigids(g_flex, g_buffers->rigidOffsets.buffer, g_buffers->rigidIndices.buffer, g_buffers->rigidLocalPositions.buffer, g_buffers->rigidLocalNormals.buffer, g_buffers->rigidCoefficients.buffer, g_buffers->rigidRotations.buffer, g_buffers->rigidTranslations.buffer, g_buffers->rigidOffsets.size() - 1, g_buffers->rigidIndices.size());

		//NvFlexSetRestParticles(g_flex, g_buffers->restPositions.buffer, g_buffers->restPositions.size());
	}

	void CreateSoftBody(float radius, Vec3 position, float clusterStiffness, int group = 0) {
		Instance instance;

		Mesh* mesh = ImportMesh(GetFilePathByPlatform(mFile).c_str());
		mesh->Normalize();
		mesh->Transform(ScaleMatrix(Vec3(1.0)));
		mesh->Transform(TranslationMatrix(Point3(position)) * ScaleMatrix(mScale * radius));

		instance.mMesh = mesh;
		instance.mColor = Vec3(0.5f, 0.1f, 1.0f);
		instance.mOffset = g_buffers->rigidTranslations.size();

		double createStart = GetSeconds();

		// create soft body definition
		NvFlexExtAsset* asset = NvFlexExtCreateSoftFromMesh(
			(float*)&instance.mMesh->m_positions[0],
			instance.mMesh->m_positions.size(),
			(int*)&instance.mMesh->m_indices[0],
			instance.mMesh->m_indices.size(),
			radius,
			mVolumeSampling,
			mSurfaceSampling,
			mClusterSpacing * radius,
			mClusterRadius * radius,
			clusterStiffness,
			mLinkRadius * radius,
			mLinkStiffness,
			mGlobalStiffness);

		double createEnd = GetSeconds();

		// create skinning
		const int maxWeights = 4;

		instance.mSkinningIndices.resize(instance.mMesh->m_positions.size() * maxWeights);
		instance.mSkinningWeights.resize(instance.mMesh->m_positions.size() * maxWeights);

		for (int i = 0; i < asset->numShapes; ++i)
			instance.mRigidRestPoses.push_back(Vec3(&asset->shapeCenters[i * 3]));

		double skinStart = GetSeconds();

		NvFlexExtCreateSoftMeshSkinning(
			(float*)&instance.mMesh->m_positions[0],
			instance.mMesh->m_positions.size(),
			asset->shapeCenters,
			asset->numShapes,
			mSkinningFalloff,
			mSkinningMaxDistance,
			&instance.mSkinningWeights[0],
			&instance.mSkinningIndices[0]);

		double skinEnd = GetSeconds();

		printf("Created soft in %f ms Skinned in %f\n", (createEnd - createStart) * 1000.0f, (skinEnd - skinStart) * 1000.0f);

		const int particleOffset = g_buffers->positions.size();
		const int indexOffset = g_buffers->rigidOffsets.back();

		cout << "num of particles: " << asset->numParticles << endl;
		cout << "num of triangles: " << asset->numTriangles << endl;
		cout << "num of shapes: " << asset->numShapes << endl;
		cout << "num of springs: " << asset->numSprings << endl;

		// add particle data to solver
		for (int i = 0; i < asset->numParticles; ++i)
		{
			g_buffers->positions.push_back(&asset->particles[i * 4]);
			g_buffers->velocities.push_back(0.0f);

			const int phase = NvFlexMakePhase(group, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);
			g_buffers->phases.push_back(phase);

			// custom density as HUE (Mobai Change)
			g_buffers->densities.push_back(asset->particleHue[i]);
		}

		// add shape data to solver
		for (int i = 0; i < asset->numShapeIndices; ++i)
		{
			g_buffers->rigidIndices.push_back(asset->shapeIndices[i] + particleOffset);
		}

		for (int i = 0; i < asset->numShapes; ++i)
		{
			g_buffers->rigidOffsets.push_back(asset->shapeOffsets[i] + indexOffset);
			g_buffers->rigidTranslations.push_back(Vec3(&asset->shapeCenters[i * 3]));
			g_buffers->rigidRotations.push_back(Quat());
			g_buffers->rigidCoefficients.push_back(asset->shapeCoefficients[i]);
		}

		// add link data to the solver
		for (int i = 0; i < asset->numSprings; ++i)
		{
			g_buffers->springIndices.push_back(asset->springIndices[i * 2 + 0]);
			g_buffers->springIndices.push_back(asset->springIndices[i * 2 + 1]);

			g_buffers->springStiffness.push_back(asset->springCoefficients[i]);
			g_buffers->springLengths.push_back(asset->springRestLengths[i]);
		}

		NvFlexExtDestroyAsset(asset);

		mInstances.push_back(instance);
	}

	virtual void Draw(int pass) {
		if (flag)
		{
			float scale = 0.3;
			Matrix33 scaleMat;
			scaleMat.cols[0] = Vec3(scale, 0, 0);
			scaleMat.cols[1] = Vec3(0, scale, 0);
			scaleMat.cols[2] = Vec3(0, 0, scale);

			Matrix33 rotMat;
			rotMat.cols[0] = Vec3(cutter.rotMat.cols[0]);
			rotMat.cols[1] = Vec3(cutter.rotMat.cols[1]);
			rotMat.cols[2] = Vec3(cutter.rotMat.cols[2]);

			rotMat = rotMat * scaleMat;

			Matrix44 modelMatrix;
			modelMatrix.SetAxis(0, rotMat.cols[0]);
			modelMatrix.SetAxis(1, rotMat.cols[1]);
			modelMatrix.SetAxis(2, rotMat.cols[2]);
			modelMatrix.SetTranslation(cutter.cutterPos.x, cutter.cutterPos.y, cutter.cutterPos.z);

			m->Transform2(modelMatrix);

			DrawMesh(m, Vec3(1, 1, 1));
		}

		//if (!g_drawMesh)
		//	return;

		//for (int s = 0; s < int(mInstances.size()); ++s)
		//{
		//	const Instance& instance = mInstances[s];

		//	Mesh m;
		//	m.m_positions.resize(instance.mMesh->m_positions.size());
		//	m.m_normals.resize(instance.mMesh->m_normals.size());
		//	m.m_indices = instance.mMesh->m_indices;

		//	for (int i = 0; i < int(instance.mMesh->m_positions.size()); ++i)
		//	{
		//		Vec3 softPos;
		//		Vec3 softNormal;

		//		for (int w = 0; w < 4; ++w)
		//		{
		//			const int cluster = instance.mSkinningIndices[i * 4 + w];
		//			const float weight = instance.mSkinningWeights[i * 4 + w];

		//			if (cluster > -1)
		//			{
		//				// offset in the global constraint array
		//				int rigidIndex = cluster + instance.mOffset;

		//				Vec3 localPos = Vec3(instance.mMesh->m_positions[i]) - instance.mRigidRestPoses[cluster];

		//				Vec3 skinnedPos = g_buffers->rigidTranslations[rigidIndex] + Rotate(g_buffers->rigidRotations[rigidIndex], localPos);
		//				Vec3 skinnedNormal = Rotate(g_buffers->rigidRotations[rigidIndex], instance.mMesh->m_normals[i]);

		//				softPos += skinnedPos * weight;
		//				softNormal += skinnedNormal * weight;
		//			}
		//		}

		//		m.m_positions[i] = Point3(softPos);
		//		m.m_normals[i] = softNormal;
		//	}

		//	DrawMesh(&m, instance.mColor);
		//}
	}

	struct Instance {
		Mesh* mMesh;
		std::vector<int> mSkinningIndices;
		std::vector<float> mSkinningWeights;
		vector<Vec3> mRigidRestPoses;
		Vec3 mColor;
		int mOffset;
	};

	std::vector<Instance> mInstances;

	const char* mFile;
	Vec3 mScale;
	Vec3 mOffset;

	float mRadius;

	float mClusterSpacing;
	float mClusterRadius;
	float mClusterStiffness;

	float mLinkRadius;
	float mLinkStiffness;

	float mGlobalStiffness;

	float mSurfaceSampling;
	float mVolumeSampling;

	float mSkinningFalloff;
	float mSkinningMaxDistance;

	float mPlasticThreshold;
	float mPlasticCreep;

	float mRelaxationFactor;

	bool mPlinth;

	int mStack[3];
};

class SoftBodyFixed : public SoftBody {
public:
	SoftBodyFixed(const char* name, const char* mesh) : SoftBody(name, mesh) {}

	virtual void Initialize() {
		SoftBody::Initialize();

		// fix any particles in the wall
		for (int i = 0; i < int(g_buffers->positions.size()); ++i)
			if (g_buffers->positions[i].x < mRadius)
				g_buffers->positions[i].w = 0.0f;
	}

	virtual void CreateBodies() {
		int x = 0;
		int y = 0;

		for (int z = 0; z < 4; ++z)
		{
			float stiffness = sqr(mClusterStiffness * (z + 1));

			CreateSoftBody(mRadius, mOffset + Vec3(x * (mScale.x + 1), y * (mScale.y + 1), -z * (mScale.z + 1)) * mRadius, stiffness, mInstances.size());
		}
	}

	virtual void PostInitialize() {
		SoftBody::PostInitialize();

		(Vec4&)g_params.planes[1] = Vec4(1.0f, 0.0f, 0.0f, 0.0f);
		g_params.numPlanes = 2;
	}
};
