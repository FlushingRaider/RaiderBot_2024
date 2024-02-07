const double Recall_T[160] = {0.0,0.19519337507740667,0.26352865468730624,0.3345655995656361,0.37070568748534133,0.40693563725828047,0.4430038746873834,0.47866813128295443,0.5137008799528627,0.5478917973676052,0.5810487823160376,0.6129983423474944,0.6435857882457678,0.6726754805631725,0.7001512705270904,0.7259172256725843,0.7498987054800678,0.7720438412835331,0.792325469801332,0.8107435644543394,0.8276465760852795,0.8433274491694608,0.8709178692564183,0.8937857899402475,0.9133773646338099,0.9328069388817957,0.9544828210543441,0.9790552368448898,1.0072998203371242,1.0231978636295245,1.0402927405280156,1.0585609160059486,1.0779532131293978,1.0983957531926465,1.11979054283465,1.142015715964096,1.1649254915732996,1.1883499758089624,1.2120950375688133,1.2359426458298108,1.2596523041438212,1.3056012847172385,1.3273059392000004,1.3478243763566202,1.3669642763651961,1.3759863534681416,1.3846408876169711,1.3929409892080153,1.4009124056030382,1.4085941464220713,1.4160380112119215,1.4233066773297216,1.4303849430278002,1.4371793695016017,1.443688524457672,1.4499815981565067,1.4561079580920597,1.4679757090301484,1.491041939862627,1.5154037746669036,1.5424835482450603,1.5724311133290496,1.6045470332942848,1.6376922558157057,1.6726868177984773,1.710849596778932,1.7514212300637568,1.7930919304633834,1.834203684158191,1.853948944166689,1.8728111963589134,1.8905507242177906,1.8989393988932042,1.9069842881894872,1.914678646054508,1.9220261375883003,1.9290427853901015,1.9357584266117323,1.9422171942347863,1.948476511492573,1.9545359406204053,1.9602875503365793,1.9657095275085619,1.9708844825877934,1.9807211172942731,1.9902430603239332,1.9951035131241712,2.0002105943547996,2.005644670057242,2.0113647065546494,2.017286217626794,2.0234436049395623,2.029883951981776,2.0366429730094047,2.043744354696547,2.0512001866531007,2.0671693194650596,2.0844713641419776,2.1029299568475124,2.122319751215163,2.1629140585977336,2.204383825443153,2.245107409388186,2.2837885876807995,2.319447449598244,2.3514036378270555,2.3792742411536407,2.403048699066493,2.4236790407370736,2.4593716240120456,2.477180075142458,2.487053952368449,2.4977634516150404,2.5093015007944586,2.5217892502249124,2.5352787141762696,2.549756789315877,2.5813572219146317,2.6155994649821785,2.633299290065193,2.6511488188561567,2.6689733886405516,2.686605724649717,2.7038894251102263,2.7206816196872348,2.7370791527217646,2.753289532008314,2.7692873438808,2.7849244832628886,2.8145835610740946,2.8414388812883176,2.853705254393232,2.865225333557095,2.876066256473563,2.886196192949545,2.9047701570060696,2.9232622042869214,2.9451577358742753,2.9724380279106306,2.988239158322966,3.0053866668132185,3.0237319626141326,3.0435563477098206,3.0652817481365107,3.088911031645425,3.114388549966514,3.1416358707701257,3.1705531395362834,3.2010199747817105,3.23289592719944,3.2660205284281396,3.3002129571418326,3.3352713666928437,3.3709719652984886,3.407068049243436,3.4432894306363466,3.479343222712316,3.5496963435584767,3.615787739553877,3.7826176777042706,};
const double Recall_T_REM[160] = {3.7826176777042706,3.587424302626864,3.5190890230169645,3.4480520781386343,3.4119119902189294,3.37568204044599,3.339613803016887,3.303949546421316,3.268916797751408,3.2347258803366654,3.201568895388233,3.169619335356776,3.1390318894585025,3.1099421971410983,3.08246640717718,3.056700452031686,3.032718972224203,3.0105738364207375,2.9902922079029386,2.9718741132499313,2.954971101618991,2.93929022853481,2.9116998084478523,2.888831887764023,2.8692403130704607,2.849810738822475,2.8281348566499265,2.803562440859381,2.7753178573671464,2.759419814074746,2.742324937176255,2.7240567616983222,2.704664464574873,2.684221924511624,2.6628271348696204,2.640601961740175,2.617692186130971,2.5942677018953084,2.570522640135457,2.54667503187446,2.5229653735604494,2.477016392987032,2.45531173850427,2.4347933013476504,2.4156534013390747,2.406631324236129,2.3979767900872995,2.3896766884962553,2.3817052721012324,2.3740235312821993,2.366579666492349,2.359311000374549,2.3522327346764706,2.345438308202669,2.338929153246599,2.332636079547764,2.326509719612211,2.314641968674122,2.2915757378416437,2.267213903037367,2.2401341294592103,2.210186564375221,2.1780706444099858,2.1449254218885647,2.109930859905793,2.071768080925339,2.0311964476405135,1.9895257472408872,1.9484139935460796,1.9286687335375816,1.9098064813453572,1.89206695348648,1.8836782788110664,1.8756333895147834,1.8679390316497626,1.8605915401159703,1.853574892314169,1.8468592510925383,1.8404004834694843,1.8341411662116975,1.8280817370838653,1.8223301273676913,1.8169081501957087,1.8117331951164772,1.8018965604099975,1.7923746173803374,1.7875141645800994,1.782407083349471,1.7769730076470287,1.7712529711496212,1.7653314600774768,1.7591740727647083,1.7527337257224946,1.745974704694866,1.7388733230077236,1.73141749105117,1.715448358239211,1.698146313562293,1.6796877208567582,1.6602979264891076,1.619703619106537,1.5782338522611177,1.5375102683160846,1.498829090023471,1.4631702281060268,1.431214039877215,1.4033434365506299,1.3795689786377774,1.358938636967197,1.323246053692225,1.3054376025618124,1.2955637253358216,1.2848542260892302,1.273316176909812,1.2608284274793582,1.247338963528001,1.2328608883883936,1.201260455789639,1.167018212722092,1.1493183876390778,1.1314688588481139,1.113644289063719,1.0960119530545538,1.0787282525940443,1.0619360580170358,1.045538524982506,1.0293281456959567,1.0133303338234705,0.997693194441382,0.968034116630176,0.941178796415953,0.9289124233110386,0.9173923441471756,0.9065514212307075,0.8964214847547254,0.877847520698201,0.8593554734173492,0.8374599418299953,0.81017964979364,0.7943785193813047,0.7772310108910521,0.758885715090138,0.73906132999445,0.7173359295677599,0.6937066460588457,0.6682291277377566,0.6409818069341449,0.6120645381679872,0.5815977029225601,0.5497217505048306,0.516597149276131,0.482404720562438,0.44734631101142686,0.41164571240578196,0.3755496284608344,0.33932824706792397,0.3032744549919544,0.23292133414579386,0.1668299381503937,0.0,};
const double Recall_ROT[160] = {-0.325953084913218,-4.683160701312832,-6.295311413168395,-7.153473268010179,-7.399185197568719,-7.557628890494267,-7.645403651165722,-7.673653998956305,-7.649455145607181,-7.576756345550435,-7.456975570281269,-7.289328903796901,-7.070952600208473,-6.796852450530508,-6.459698197578316,-6.049472176167179,-5.552985802170897,-4.953307108860664,-4.229224985864672,-3.3550698166634367,-2.3016305285882286,-1.0397494935934595,2.1588395331070447,5.810438998795588,7.775103957434068,9.765216465536033,13.878421110501792,17.977371785965644,21.19665432821605,22.427280381690547,23.419068802594705,24.188392590611684,24.750265818905387,25.11590260230921,25.29134100256854,25.276472572655024,25.064046150470976,24.638325976599216,23.973108299304318,23.02875863180446,21.747842984428058,17.81741830472782,14.895561006305034,11.07096143575381,6.076994796542331,3.051166315068488,-0.3688056495215372,-4.198969554328175,-8.43137433386913,-13.022041800823846,-17.880361960837167,-22.86478531949076,-27.78943182426693,-32.442374423593485,-36.61018697041735,-40.099358990645655,-42.74693453728998,-45.000000051421985,-42.72559274830192,-38.297658851417864,-34.22911877526833,-31.253570270821125,-29.30343771788746,-28.17228665153727,-27.712353870811867,-27.870298825401985,-28.699061077446736,-30.393792283867054,-33.3878223769258,-35.622102017452534,-38.569034553278385,-42.47468258659421,-44.88343199988482,-47.65356606664473,-50.82632730232344,-54.43306929177874,-58.483366466685304,-62.94887497854721,-67.74553056673156,-72.7203213812665,-77.65118850499913,-82.26553832473537,-86.2733448495795,-89.40131938019728,-92.12109650193781,-94.72009991559845,-97.68416738307879,-101.44473284604429,-105.72959997292568,-110.26670729842559,-114.81487029826782,-119.18747668145846,-123.261561861138,-126.97267525961402,-130.30147869372652,-133.25855455233344,-138.17546067238544,-142.00330285227227,-145.01128130481064,-147.4134938675791,-150.99469768033566,-153.57206401915207,-155.59752676209442,-157.3405357060503,-158.984338552437,-160.67472408048877,-162.54112690253933,-164.68362896397218,-167.08063617362654,-170.39669691770743,-174.7067978059018,-178.85441209676918,176.72085406680637,172.53654549553164,168.8534414706364,165.74923290424994,163.20192013430417,159.5183704518162,157.27673866344998,156.56923879671425,156.09196304627898,155.82326562575514,155.74993619258055,155.8663396650631,156.17391072696935,156.68097350194404,157.40280947255408,158.3618257350364,159.5875347970969,162.9861349230392,167.88535815573678,170.91740874947033,174.2374485124552,177.62949612720868,-179.27925938674858,-176.13453903141234,-176.3384829664001,-176.69587452830322,-176.99302269887022,-177.10898087090075,-177.20691283925902,-177.2903776614157,-177.36258098925435,-177.4262275838403,-177.4835585931127,-177.53644675796087,-177.58650111020427,-177.63516654319963,-177.6838176690582,-177.73385242197438,-177.78679433692983,-177.84441600082312,-177.90890159685006,-177.98307556526746,-178.07073972063586,-178.17718631238534,-178.30999330772514,-178.70444438093543,-179.4210563869206,178.2100895953895,};
const double Recall_Y[160] = {90.04328017782359,94.3817752747633,97.93745285895808,102.74596171529659,105.62759455117111,108.8113840491039,112.2746702793898,115.98768988451761,119.91471537092916,124.01519440077836,128.24488908369037,132.55701526852056,136.9033818351137,141.23552998606294,145.505872538469,149.66883321569915,153.68198593914636,157.50719411998838,161.11174995094677,164.46951369804592,167.56205299237254,170.37978212183395,175.20353607245167,179.0843160764268,182.38012384200513,185.6140821255143,189.10820306960522,193.0146119200048,197.3838351294429,199.7344124036329,202.18268658318033,204.71436377395145,207.31215382453658,209.9563292707982,212.62528428041819,215.296093597446,217.94507148684627,220.54833067904661,223.08234131448515,225.52448988815863,227.8536381941699,232.09911134243467,233.98556676935453,235.70040098704052,237.23823645334267,237.94040430966095,238.59852459250368,239.21337495773994,239.7861047397068,240.31825241822642,240.81176308562334,241.26900591374132,241.69279162096066,242.08638993921528,242.45354708100982,242.79850320643666,243.12600989019325,243.7503431066126,245.03088069801515,246.5651127687496,248.50485136330215,250.9058766637159,253.74456086271903,256.93449203685316,260.34309801960177,263.808270274518,267.1549877683532,270.21194084418505,272.82815509454554,273.9334571088907,274.8896152345496,275.69087977800024,276.03281579662075,276.335888975023,276.6006999364514,276.8281884867361,277.0196498485442,277.1767508956307,277.3015463870898,277.396495201606,277.46447657170495,277.5088063180048,277.53325308346734,277.5420545676488,277.53211517887337,277.5070189276946,277.4809550644592,277.43950329191273,277.37845759334067,277.29405558638524,277.1829635207115,277.042261275674,276.86942735798306,276.66232389937136,276.4191816542601,276.1385849974255,275.4610440354654,274.62487833012636,273.6299629743749,272.48039076283436,269.7518550111132,266.53812462743986,262.9748116116114,259.2189240211525,255.4335035817223,251.77226329752202,248.36422506170163,245.29835726676734,242.60821241498869,238.12004776123618,235.96943862473066,234.79818928616731,233.52499203282642,232.13009381760068,230.6001031653562,228.9274678136968,227.10995235372835,223.05478958538185,218.5032190082408,216.07729659937448,213.57548343516947,211.01813602264284,208.42674884842728,205.82343201953532,203.23038890412354,200.6693937722567,198.16126943667226,195.7253648935441,193.3790329632472,189.01138318823678,185.13736950769865,183.39276164770885,181.770671513815,180.2598526371964,178.84288349934695,176.18679896208994,173.46723319042084,170.14539995938446,165.85256501025697,163.28815790472248,160.44066633504877,157.31991018578756,153.94481381154105,150.34223415055675,146.54578883832232,142.59468432116066,138.5325439698246,134.40623619309181,130.26470255135982,126.1577858702407,122.1350583541562,118.2446496999324,114.5320752103947,111.03906390796271,107.80238664824522,104.85268423363476,102.21329552690295,97.91527367162502,94.90446150876643,91.7323687814368,};
const double Recall_X[160] = {279.3086045539113,279.1209430540658,278.77151519705666,278.19884066838983,277.8303754593273,277.4120069751031,276.9494486773249,276.4497850935775,275.92122974199486,275.3728830558316,274.8144903080348,274.2561995358156,273.70831946522094,273.18107743570533,272.6843773247024,272.2275574721967,271.81914860529525,271.4666317627996,271.1761962197769,270.95249741213223,270.7984148611798,270.7148100982151,270.75093765876534,271.0182136774907,271.42619107038377,271.9086785935797,272.6382967234645,273.7605740860684,275.32542713792753,276.2673963092424,277.3039091564304,278.4217728486521,279.6049592301646,280.83512822626983,282.0921512492631,283.3546346043813,284.6004428957512,285.80722243233777,286.95292463389274,288.0163294369025,288.97756870053684,290.52397809546454,291.0808822120486,291.4801355717353,291.71648073633554,291.7731338634054,291.78915262603346,291.7652291915269,291.7024019253348,291.60207174748405,291.466018489015,291.2964172484179,291.09585474806806,290.8673456906622,290.6143491156542,290.340784755691,290.05104939304863,289.4431361755897,288.19883605026234,286.88871268982376,285.47312008987177,283.93710320795265,282.286145344854,280.54191552589765,278.7380158822322,276.91572903212614,275.1197654622603,273.3940109090209,271.77727373979195,271.0194794030176,270.29903233424864,269.61746182510836,269.29143201665,268.9751824656496,268.6685690207666,268.3713612500205,268.083238287843,267.8037846821298,267.53248624129276,267.2687258813121,267.0117794727881,266.76081168799345,266.5148718479252,266.2728897693567,265.7958957250068,265.3185917693052,265.0759211364135,264.8287237881524,264.57581859429143,264.31618108158375,264.0489375954004,263.7733594613656,263.48885714699173,263.1949744233142,262.8913825265267,262.57787431961583,261.9208535311458,261.22446563978843,260.4908366007564,259.7234779149104,258.1074243683758,256.42500846817154,254.73514544110276,253.10098378701832,251.58392679303668,250.2376540477713,249.10214295555596,248.19769025067077,247.51893351156747,246.65289155072523,246.34853161308294,246.28210424492727,246.30661409854298,246.4394658163474,246.69229963468578,247.07146902782424,247.57851835194194,248.9612544913537,250.77483101233696,251.80956128048194,252.90719420044263,254.04898433358247,255.2151982751184,256.38559229811375,257.53988999747037,258.6582599339217,259.7217932780248,260.7129814541532,261.61619378448944,263.10842354951563,264.1291455784597,264.45718001536517,264.6696384571446,264.77740954243467,264.79708095962144,264.66983665593165,264.48969315014904,264.28797726947005,264.0523435329202,263.9203597258667,263.77910633761985,263.6292220664058,263.4716935519815,263.3078070504218,263.13910010890703,262.9673132405104,262.79434159898534,262.6221866535529,262.45290786368923,262.2885743539132,262.13121658857335,261.9827780466357,261.84506689647105,261.7197076706424,261.60809294069225,261.51133499193026,261.4302174982204,261.3161055629101,261.2636179386843,261.2916594487052,};
