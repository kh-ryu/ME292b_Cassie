function model = urdf_to_featherstone(urdf_file, xsl_file, output_file)
    if(nargin == 1)
        xsl_file = 'urdf_to_featherstone.xsl' ;
        output_file = 'gen_model_autogen.m' ;
    end
    xsl_output = xslt(urdf_file, xsl_file, output_file) ;
    
    model = [] ;
end