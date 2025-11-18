<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<xsl:template match="/robot">

      model.link_names = {
      <xsl:for-each select="link">'<xsl:value-of select="@name"/>',</xsl:for-each>
      } ;
      model.link_id = [
      <xsl:for-each select="link"><xsl:value-of select="position()"/>,</xsl:for-each>
      ] ;
     model.link_map = containers.Map(model.link_names, model.link_id) ;

      <!--model.Xtree(1:length(model.link_id)).axis = [0 0 0] ;-->
      <xsl:for-each select="joint">
        model.parent(model.link_map('<xsl:value-of select="child/@link"/>')) = model.link_map('<xsl:value-of select="parent/@link"/>') ;
        <xsl:if test="@type='fixed'">
        	model.jtype{model.link_map('<xsl:value-of select="child/@link"/>')} = axis_fcn([0, 0, 0]) ;
        </xsl:if>
        <xsl:if test="not(@type='fixed')">
		<xsl:if test="axis">
			model.jtype{model.link_map('<xsl:value-of select="child/@link"/>')} = axis_fcn([<xsl:value-of select="axis/@xyz"/>]) ;
		</xsl:if>
		<xsl:if test="not(axis)">
			model.jtype{model.link_map('<xsl:value-of select="child/@link"/>')} = axis_fcn([0, 0, 0]) ;
		</xsl:if>
	</xsl:if>
	
	model.jnames{model.link_map('<xsl:value-of select="child/@link"/>')} = '<xsl:value-of select="@name"/>' ;
	
        model.Xtree{model.link_map('<xsl:value-of select="child/@link"/>')} = xtree_fcn(...
            [<xsl:value-of select="origin/@xyz"/>], ...
            <xsl:if test="origin/@rpy">[<xsl:value-of select="origin/@rpy"/>] ...</xsl:if><xsl:if test="not(origin/@rpy)">[0, 0, 0] ...</xsl:if>
        ) ;
      </xsl:for-each>
      
      <xsl:for-each select="link">
        <xsl:if test="inertial">
          model.I{<xsl:value-of select="position()"/>} = mcI_fcn(<xsl:value-of select="inertial/mass/@value"/>, ...
              [<xsl:value-of select="inertial/origin/@xyz"/>], ...
              [<xsl:value-of select="inertial/origin/@rpy"/>], ...
              inertia_fcn(<xsl:value-of select="inertial/inertia/@ixx"/>, ...
                         <xsl:value-of select="inertial/inertia/@ixy"/>, ...
                         <xsl:value-of select="inertial/inertia/@ixz"/>, ...
                         <xsl:value-of select="inertial/inertia/@iyy"/>, ...
                         <xsl:value-of select="inertial/inertia/@iyz"/>, ...
                         <xsl:value-of select="inertial/inertia/@izz"/>)) ;
        </xsl:if>
        <xsl:if test="not(inertial)">
          model.I{<xsl:value-of select="position()"/>} = mcI(0, [0,0,0], zeros(3,3)) ;
        </xsl:if>
      </xsl:for-each>
      
      
      model = update_model_breadth_first(model) ;
      model.NB = length(model.link_id) ;
      model.Xtree{1} = eye(6) ;
      model.jtype{1} = 'R' ;
      
      
	for j=1:length(model.link_id)
	    eval(['model.idx.' model.link_names{j} ' = ' num2str(model.link_id(j)) ';']) ;
	end

</xsl:template>



</xsl:stylesheet>
