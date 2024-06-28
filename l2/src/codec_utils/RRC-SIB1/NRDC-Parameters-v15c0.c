/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "NRDC-Parameters-v15c0.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_pdcp_DuplicationSplitSRB_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_pdcp_DuplicationSplitDRB_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_pdcp_DuplicationSplitSRB_value2enum_2[] = {
	{ 0,	9,	"supported" }
};
static const unsigned int asn_MAP_pdcp_DuplicationSplitSRB_enum2value_2[] = {
	0	/* supported(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_pdcp_DuplicationSplitSRB_specs_2 = {
	asn_MAP_pdcp_DuplicationSplitSRB_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_pdcp_DuplicationSplitSRB_enum2value_2,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_pdcp_DuplicationSplitSRB_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_pdcp_DuplicationSplitSRB_2 = {
	"pdcp-DuplicationSplitSRB",
	"pdcp-DuplicationSplitSRB",
	&asn_OP_NativeEnumerated,
	asn_DEF_pdcp_DuplicationSplitSRB_tags_2,
	sizeof(asn_DEF_pdcp_DuplicationSplitSRB_tags_2)
		/sizeof(asn_DEF_pdcp_DuplicationSplitSRB_tags_2[0]) - 1, /* 1 */
	asn_DEF_pdcp_DuplicationSplitSRB_tags_2,	/* Same as above */
	sizeof(asn_DEF_pdcp_DuplicationSplitSRB_tags_2)
		/sizeof(asn_DEF_pdcp_DuplicationSplitSRB_tags_2[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_pdcp_DuplicationSplitSRB_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_pdcp_DuplicationSplitSRB_specs_2	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_pdcp_DuplicationSplitDRB_value2enum_4[] = {
	{ 0,	9,	"supported" }
};
static const unsigned int asn_MAP_pdcp_DuplicationSplitDRB_enum2value_4[] = {
	0	/* supported(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_pdcp_DuplicationSplitDRB_specs_4 = {
	asn_MAP_pdcp_DuplicationSplitDRB_value2enum_4,	/* "tag" => N; sorted by tag */
	asn_MAP_pdcp_DuplicationSplitDRB_enum2value_4,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_pdcp_DuplicationSplitDRB_tags_4[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_pdcp_DuplicationSplitDRB_4 = {
	"pdcp-DuplicationSplitDRB",
	"pdcp-DuplicationSplitDRB",
	&asn_OP_NativeEnumerated,
	asn_DEF_pdcp_DuplicationSplitDRB_tags_4,
	sizeof(asn_DEF_pdcp_DuplicationSplitDRB_tags_4)
		/sizeof(asn_DEF_pdcp_DuplicationSplitDRB_tags_4[0]) - 1, /* 1 */
	asn_DEF_pdcp_DuplicationSplitDRB_tags_4,	/* Same as above */
	sizeof(asn_DEF_pdcp_DuplicationSplitDRB_tags_4)
		/sizeof(asn_DEF_pdcp_DuplicationSplitDRB_tags_4[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_pdcp_DuplicationSplitDRB_constr_4,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_pdcp_DuplicationSplitDRB_specs_4	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_NRDC_Parameters_v15c0_1[] = {
	{ ATF_POINTER, 2, offsetof(struct NRDC_Parameters_v15c0, pdcp_DuplicationSplitSRB),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_pdcp_DuplicationSplitSRB_2,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"pdcp-DuplicationSplitSRB"
		},
	{ ATF_POINTER, 1, offsetof(struct NRDC_Parameters_v15c0, pdcp_DuplicationSplitDRB),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_pdcp_DuplicationSplitDRB_4,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"pdcp-DuplicationSplitDRB"
		},
};
static const int asn_MAP_NRDC_Parameters_v15c0_oms_1[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_NRDC_Parameters_v15c0_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_NRDC_Parameters_v15c0_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* pdcp-DuplicationSplitSRB */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* pdcp-DuplicationSplitDRB */
};
asn_SEQUENCE_specifics_t asn_SPC_NRDC_Parameters_v15c0_specs_1 = {
	sizeof(struct NRDC_Parameters_v15c0),
	offsetof(struct NRDC_Parameters_v15c0, _asn_ctx),
	asn_MAP_NRDC_Parameters_v15c0_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_NRDC_Parameters_v15c0_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_NRDC_Parameters_v15c0 = {
	"NRDC-Parameters-v15c0",
	"NRDC-Parameters-v15c0",
	&asn_OP_SEQUENCE,
	asn_DEF_NRDC_Parameters_v15c0_tags_1,
	sizeof(asn_DEF_NRDC_Parameters_v15c0_tags_1)
		/sizeof(asn_DEF_NRDC_Parameters_v15c0_tags_1[0]), /* 1 */
	asn_DEF_NRDC_Parameters_v15c0_tags_1,	/* Same as above */
	sizeof(asn_DEF_NRDC_Parameters_v15c0_tags_1)
		/sizeof(asn_DEF_NRDC_Parameters_v15c0_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_NRDC_Parameters_v15c0_1,
	2,	/* Elements count */
	&asn_SPC_NRDC_Parameters_v15c0_specs_1	/* Additional specs */
};

