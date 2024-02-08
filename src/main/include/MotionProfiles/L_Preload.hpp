const double L_Preload_T[62] = {0.0,0.16853488156841254,0.20513967237857672,0.2349891542101904,0.26027677770358093,0.27160775818619654,0.2822121302936612,0.2921814087274988,0.30159608551092504,0.31052849168440344,0.3190445697272099,0.32720489081421344,0.33506514266568843,0.342676258677758,0.3500843296540683,0.3573304188811104,0.3644503797766814,0.3714747492729474,0.3784287607605357,0.38533249188691693,0.3922011390880837,0.39904539510583115,0.4058718982870572,0.4194781475142963,0.433014236082125,0.44643661190734263,0.459680714929772,0.48532710127142553,0.5093594980002771,0.5312895920610746,0.5680228818782147,0.59801081061427,0.6287420138398413,0.6659853160173563,0.7106329616980549,0.7350277861857256,0.7475348323377967,0.7601633938805672,0.7728612093740936,0.7855946821799603,0.7919736282284366,0.7983632952583248,0.8047699498091544,0.8112033390863038,0.8176770733758127,0.8242089527207149,0.8308212134835243,0.8375406761590507,0.8443987878677226,0.851431571271722,0.8586795145013866,0.8661874610879271,0.8740045819449664,0.8821845321333266,0.8907859162302403,0.8998732154344787,0.9095183814040755,0.919803400537803,0.9426932074119542,0.9696296938452383,1.0025853418897754,1.1539849306398893,};
const double L_Preload_T_REM[62] = {1.1539849306398893,0.9854500490714767,0.9488452582613125,0.9189957764296989,0.8937081529363083,0.8823771724536927,0.8717728003462281,0.8618035219123905,0.8523888451289643,0.8434564389554858,0.8349403609126793,0.8267800398256758,0.8189197879742008,0.8113086719621312,0.803900600985821,0.7966545117587789,0.7895345508632079,0.782510181366942,0.7755561698793536,0.7686524387529723,0.7617837915518055,0.7549395355340581,0.748113032352832,0.7345067831255929,0.7209706945577643,0.7075483187325466,0.6943042157101172,0.6686578293684637,0.6446254326396121,0.6226953385788146,0.5859620487616746,0.5559741200256193,0.525242916800048,0.48799961462253294,0.4433519689418344,0.41895714445416365,0.40645009830209256,0.3938215367593221,0.38112372126579563,0.36839024845992896,0.36201130241145263,0.35562163538156444,0.3492149808307349,0.34278159155358545,0.3363078572640765,0.32977597791917435,0.32316371715636494,0.31644425448083857,0.3095861427721667,0.3025533593681673,0.2953054161385027,0.28779746955196217,0.27998034869492283,0.2718003985065627,0.263199014409649,0.25411171520541054,0.2444665492358138,0.2341815301020863,0.2112917232279351,0.18435523679465093,0.15139958875011383,0.0,};
const double L_Preload_ROT[62] = {0.0,-2.074478053815079,-4.560631132581361,-7.9923501345967765,-12.386914564822789,-14.948116474203239,-17.748365845306132,-20.779395154854342,-24.026847105375726,-27.469224882218587,-31.077371769744065,-34.814716120976506,-38.63842883861288,-42.501473751753906,-46.355325288903835,-50.152952846535364,-53.851597795813845,-57.41492905728244,-60.81433066527005,-64.02928163741988,-67.04696271172853,-69.86132323370549,-72.47186054392239,-77.09951163768652,-80.99028345553764,-84.22421778945842,-86.88238597658516,-90.75009570125853,-93.02892970187688,-93.97805252041752,-92.55369343027841,-90.00000010283986,-88.3680866436218,-87.779144637167,-91.2402620166403,-95.08429441506993,-97.69887137953856,-100.86460943161701,-104.65795625053563,-109.15344542223076,-111.6833103166085,-114.40678902671966,-117.32345568818988,-120.42785872712349,-123.7084299062199,-127.14663123550675,-130.7165529958144,-134.38516498755894,-138.1133483138338,-141.85769688571565,-145.57290654511672,-149.21442039869873,-152.74092872835115,-156.1163596224048,-159.31112637324563,-162.30256824280625,-165.07467348140895,-167.61726822322086,-171.99548588354583,-175.427208079962,-177.9191343721777,-180.00000020567958,};
const double L_Preload_X[62] = {92.28346752,95.52061365577283,97.07713498384152,98.56579921335171,99.97117202475631,100.6387034137564,101.28109730835722,101.89730978849386,102.48647391620429,103.04789275912631,103.58103241399425,104.08551503013597,104.56111183296957,105.0077361475003,105.4254364218174,105.81438925059089,106.17489239856849,106.5073578240724,106.81230470249616,107.09035244980149,107.34221374601512,107.56868755872564,107.77065216658042,108.10492157858643,108.35336932526562,108.52516523058769,108.6300783511582,108.67998913904084,108.5847553666378,108.4213305553803,108.13135748515315,108.06022528302965,108.1049164396255,108.26971185789183,108.34275380586912,108.21658795170043,108.08655874847962,107.90187322644096,107.65537170330776,107.34037774892934,107.1552820021523,106.95086128287312,106.72648958137994,106.48160167201739,106.21569820998634,105.9283508281436,105.61920723380176,105.2879963055289,104.93453318994838,104.55872439853857,104.1605729044327,103.7401832392185,103.29776658973807,102.83364589488762,102.34826094241718,101.84217346573045,101.31607224068449,100.77077818238948,99.62658650355765,98.41900221357085,97.15986952000036,94.54751645412505,};
const double L_Preload_Y[62] = {277.05648641576056,277.0170895546765,276.92919961801505,276.7678925417953,276.5178029219667,276.35579386120816,276.16766977586724,275.9526128246445,275.710022428342,275.4395054538045,275.14086639786086,274.814097571265,274.45936928263717,274.0770200224052,273.6675466467456,273.23159456152496,272.76994790624076,272.283519737963,271.773342215275,271.24055678221504,270.686404352217,270.112215492052,269.5194006057695,268.2838746610882,266.9923334858994,265.6579332166184,264.2941650444678,261.5322803471936,258.8093736924945,256.2133969672696,251.6200189299457,247.7789506198007,243.9921096415344,239.69419839729858,234.95532066214554,232.56104907866356,231.39163142072925,230.2546563573151,229.16054038799317,228.11936495752818,227.62162931664326,227.14063482981484,226.6774398443678,226.23303646181486,225.80834298704187,225.40419637749372,225.02134469235955,224.66043954175834,224.32202853592455,224.00654773439365,223.71431409518746,223.4455179239999,223.20021532338257,222.9783206419299,222.77959892346516,222.60365835622574,222.4499427220486,222.31772384555615,222.11395857315358,221.98281106075117,221.91149526252,221.87959203106695,};
