#!/bin/bash

mkdir koper-data/
cd koper-data 
wget https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.110/Bilder/Forschung/Datensaetze/20140618_Sequence1a.zip 
wget https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.110/Bilder/Forschung/Datensaetze/20140527_Sequence1b.zip 
wget https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.110/Bilder/Forschung/Datensaetze/20140527_Sequence1c.zip
wget https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.110/Bilder/Forschung/Datensaetze/20140527_Sequence1d.zip
wget https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.110/Bilder/Forschung/Datensaetze/20140527_Sequence2.zip 
wget https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.110/Bilder/Forschung/Datensaetze/20140527_Sequence3.zip 
wget https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.110/Bilder/Forschung/Datensaetze/20140618_DataSetViewer.zip
wget https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.110/Bilder/Forschung/Datensaetze/20141010_DatasetDocumentation.pdf 

unzip 20140618_Sequence1a.zip 
unzip 20140527_Sequence1b.zip 
unzip 20140527_Sequence1c.zip
unzip 20140527_Sequence1d.zip
unzip 20140527_Sequence2.zip 
unzip 20140527_Sequence3.zip  
unzip 20140618_DataSetViewer.zip

echo("----- download completed -----")
